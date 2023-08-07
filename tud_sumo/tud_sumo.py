import os, sys, traci, json, math
from functools import reduce
from tqdm import tqdm
from random import randint
from copy import copy
from warnings import warn

class Simulation:
    def __init__(self, all_vtypes=[], verbose = False) -> None:
        path_tools = os.path.join(os.environ.get("SUMO_HOME"), 'tools')

        if path_tools in sys.path: pass
        else: sys.path.append(path_tools)

        self.running = False
        self.curr_step = 0
        self.gui = False
        self.verbose = verbose or '-v' in sys.argv

        self.junc_phases = None

        self.track_juncs = False
        self.all_juncs = []
        self.tracked_juncs = {}

        self.available_detectors = {}
        self.step_length = None

        self.get_individual_vehicle_data = True
        self.all_curr_vehicle_ids = []
        self.known_vehicles = {}
        self.light_changes = 0

        self.all_data = None

        self.all_vtypes = set(all_vtypes)

    def start(self, config_file = None, net_file = None, route_file = None, add_file = None, cmd_options = None, step_len = 0.5, warnings = False, gui = False) -> None:
        """
        Intialises SUMO simulation.
        :param config_file: Location of '.sumocfg' file (can be given instead of net_file)
        :param net_file:    Location of '.net.xml' file (can be given instead of config_file)
        :param route_file:  Location of '.rou.xml' route file
        :param add_file:    Location of '.add.xml' additional file
        :param cmd_options: List of any other command line options
        :param step_len:    Simulation step length
        :param warnings:    Print warnings
        :param gui:         Bool denoting whether to run GUI
        """

        self.gui = gui
        sumoCMD = ["sumo-gui"] if gui else ["sumo"]

        if config_file == net_file == None: raise ValueError("Simulation.start: Either config or network file required.")
        
        if config_file != None:
            if config_file.endswith(".sumocfg"): sumoCMD += ["-c", config_file]
            else: raise ValueError("Simulation.start: Invalid config file extension.")
        else:
            sumoCMD += ["-n", net_file]
            if route_file != None: sumoCMD += ["-r", route_file]
            if add_file != None: sumoCMD += ["-a", add_file]
            if cmd_options != None: sumoCMD += cmd_options
        
        self.step_length = step_len
        sumoCMD += ["--step-length", str(self.step_length)]
        if not warnings or not self.verbose: sumoCMD.append("--no-warnings")

        traci.start(sumoCMD)
        self.running = True

        if self.junc_phases != None: self.update_lights()
        self.time_val = traci.simulation.getTime()

        self.available_detectors = {detector_id: 'multientryexit' for detector_id in list(traci.multientryexit.getIDList())}
        self.available_detectors.update({detector_id: 'inductionloop' for detector_id in list(traci.inductionloop.getIDList())})
    
    def start_junc_tracking(self, juncs=None):
        """
        Initalise junctions and start tracking states and flows.
        :param juncs: Either junc_id|list of junc_ids, or dict containing juncs parameters. Defaults to all junctions with traffic lights.
        """

        self.track_juncs = True

        all_juncs = list(traci.junction.getIDList())
        all_tls = list(traci.trafficlight.getIDList())

        if juncs == None: # If none given, track all junctions with traffic lights
            track_list, junc_params = all_tls, None
        else:
            
            if isinstance(juncs, dict):
                junc_ids, junc_params = list(juncs.keys()), juncs
            elif isinstance(juncs, list) or isinstance(juncs, tuple):
                junc_ids, junc_params = juncs, None
            elif isinstance(juncs, str):
                junc_ids, junc_params = [juncs], None
            else: raise TypeError("Simulation.start_junc_tracking: Invalid junc_params (must be [str|list|dict], not '"+type(juncs).__name__+"')")

            if len(set(all_juncs).intersection(set(junc_ids))) != len(junc_ids):
                raise KeyError("Simulation.start_junc_tracking: Junction ID not found")
            else: track_list = junc_ids

        self.all_juncs, self.all_tls = all_juncs, all_tls

        for junc_id in track_list:
            junc_param = junc_params[junc_id] if junc_params != None else None
            self.tracked_juncs[junc_id] = Junction(junc_id, self, junc_param)

    def reset_data(self):
        """
        Resets data collection.
        """

        for junction in self.tracked_juncs.values():
            junction.reset()

        self.all_data = None

    def is_running(self, close=True):
        if traci.simulation.getMinExpectedNumber() == 0:
            if close: self.end()
            print('Ended simulation: no vehicles remaining.')
            return False
        
        return True

    def end(self):
        traci.close()
        self.running = False

    def save_data(self, filename, overwrite=True):
        """
        Save all vehicle, detector and junction data in a JSON file.
        :param filename:  Output filepath
        :param overwrite: Prevent previous outputs being overwritten
        """
        if os.path.exists(filename) and overwrite and self.verbose:
            print("Warning: Simulation.save_data: File '{0}' already exists and will be overwritten".format(filename))
        elif os.path.exists(filename) and not overwrite:
            raise FileExistsError("Simulation.save_data: File '{0}' already exists and can't be overwritten".format(filename))
        
        if not filename.endswith(".json"): filename += ".json"

        if self.all_data != None:
            with open(filename, "w") as fp:
                json.dump(self.all_data, fp, indent=4)
        else: raise AssertionError("Simulation.save_data: No data to save as simulation has not been run")

    def step_through(self, n_steps = 1, end_step = None, sim_dur = None, n_light_changes = None, detector_list = None, vTypes = None, keep_data=True, append_data = True, cumulative_data = False) -> dict:
        """
        Step through simulation from the current time until end_step, aggregating data during this period.
        :param n_steps:         Perform n steps of the simulation (defaults to 1)
        :param end_step:        End point for stepping through simulation (given instead of end_step)
        :param sim_dur:         Simulation duration
        :param n_light_changes: Run for n light changes (across all junctions)
        :param detector_list:   List of detector IDs to collect data from (defaults to all)
        :param vTypes:          Vehicle type(s) to collect data of (list of types or string, defaults to all)
        :param cumulative_data: Denotes whether to get cumulative veh count and TTS values
        :param keep_data:       Denotes whether to store data collected during this run (defaults to True)
        :param append_data:     Denotes whether to append simulation data to that of previous runs (defaults to True)
        :return dict:           All data collected through the time period, separated by detector
        """

        if append_data == True: prev_data = self.all_data
        else: prev_data = None

        if detector_list == None: detector_list = list(self.available_detectors.keys())
        start_time = self.curr_step

        if end_step == None and n_steps != None: end_step = self.curr_step + n_steps
        elif end_step == None and sim_dur != None: end_step = self.curr_step + (sim_dur / self.step_length)
        elif end_step == None and n_light_changes != None: end_step, init_changes = math.inf, self.light_changes
        elif end_step == None: raise ValueError("Simulation.step_through: No time value given")

        if prev_data == None:
            prev_steps, all_data = 0, {"data": {"detector": {}, "junctions": {}, "vehicle": {}, "all_vehicles": []}, "step_len": self.step_length, "start": start_time}
        else: 
            prev_steps = set([len(data_arr) for data_arr in prev_data["data"]["vehicle"].values()] +
                            [len(detector_data["speeds"]) for detector_data in prev_data["data"]["detector"].values()] +
                            [len(detector_data["veh_counts"]) for detector_data in prev_data["data"]["detector"].values()])
            
            if len(prev_steps) != 1: raise ValueError("Simulation.step_through: Invalid prev_data (different length arrays)")
            else:
                prev_steps = prev_steps.pop()
                all_data = prev_data

        if self.verbose: pbar = tqdm(desc="Running simulation", total=end_step - self.curr_step)
        while self.curr_step < end_step:

            last_step_data, all_v_data = self.step(detector_list, vTypes)
            all_data["data"]["all_vehicles"].append(all_v_data)
            all_data["data"]["junctions"] = last_step_data["junctions"]

            if len(all_data["data"]["detector"]) == 0:
                for detector_id in detector_list:
                    all_data["data"]["detector"][detector_id] = {"type": self.available_detectors[detector_id], "speeds": [], "veh_counts": [], "occupancies": []}
                all_data["data"]["vehicle"] = {"no_vehicles": [], "tts": [], "delay": []}

            for detector_id in last_step_data["detector"].keys():
                if detector_id not in all_data["data"]["detector"].keys(): raise KeyError("Simulation.step_through: Unrecognised detector ID found ('{0}'}".format(detector_id))
                for data_key, data_val in last_step_data["detector"][detector_id].items():
                    all_data["data"]["detector"][detector_id][data_key].append(data_val)

            for data_key, data_val in last_step_data["vehicle"].items():
                all_data["data"]["vehicle"][data_key].append(data_val)

            if self.verbose: pbar.update(1)

            if end_step == math.inf and n_light_changes != None:
                if self.light_changes - init_changes >= n_light_changes: break

        if cumulative_data:
            for detector_data in all_data["data"]["detector"].values():
                detector_data["veh_counts"] = get_cumulative_arr(detector_data["veh_counts"], prev_steps)
            all_data["data"]["vehicle"]["no_vehicles"] = get_cumulative_arr(all_data["data"]["vehicle"]["no_vehicles"], prev_steps)
            all_data["data"]["vehicle"]["tts"] = get_cumulative_arr(all_data["data"]["vehicle"]["tts"], prev_steps)

        all_data["end"] = self.curr_step

        if keep_data:
            self.all_data = all_data
            return all_data
        else:
            self.reset_data()
            return None

    def step(self, detector_list = None, vTypes = None) -> dict:
        """
        Increment simulation by one time step, updating light state. step_through is recommended to run the simulation.
        :param detector_list: List of detector IDs to collect data from
        :param vTypes:        Vehicle type(s) to collect data of (list of types or string, defaults to all)
        :return dict:         Simulation data
        """

        data = {"detector": {}, "vehicle": {}, "junctions": {}}
        
        traci.simulationStep()
        time_diff = traci.simulation.getTime() - self.time_val
        self.time_val = traci.simulation.getTime()

        self.all_curr_vehicle_ids = list(traci.vehicle.getIDList())

        if self.junc_phases != None:
            update_junc_lights = []
            for junction_id, phases in self.junc_phases.items():
                phases["curr_time"] += time_diff
                if phases["curr_time"] >= phases["cycle_len"]:
                    phases["curr_time"] = 0
                    phases["curr_phase"] = 0
                    update_junc_lights.append(junction_id)
                elif phases["curr_time"] >= sum(phases["times"][:phases["curr_phase"] + 1]):
                    phases["curr_phase"] += 1
                    update_junc_lights.append(junction_id)

            self.update_lights(update_junc_lights)

        if detector_list == None: detector_list = list(self.available_detectors.keys())
        for detector_id in detector_list:
            data["detector"][detector_id] = {}
            if detector_id not in self.available_detectors.keys(): raise KeyError("Simulation.step_through: Unrecognised detector ID found ('{0}'}".format(detector_id))
            if self.available_detectors[detector_id] == "multientryexit":
                data["detector"][detector_id]["speeds"] = traci.multientryexit.getLastStepMeanSpeed(detector_id)
                data["detector"][detector_id]["veh_counts"] = traci.multientryexit.getLastStepVehicleNumber(detector_id)
            elif self.available_detectors[detector_id] == "inductionloop":
                data["detector"][detector_id]["speeds"] = traci.inductionloop.getLastStepMeanSpeed(detector_id)
                data["detector"][detector_id]["veh_counts"] = traci.inductionloop.getLastStepVehicleNumber(detector_id)
                data["detector"][detector_id]["occupancies"] = traci.inductionloop.getLastStepOccupancy(detector_id)
            else:
                warn("Warning: Unknown detector type '"+self.available_detectors[detector_id]+"'")

        total_v_data, all_v_data = self.get_all_vehicle_data(types=vTypes)
        data["vehicle"]["no_vehicles"] = total_v_data["no_vehicles"]
        data["vehicle"]["tts"] = total_v_data["no_vehicles"] * self.step_length
        data["vehicle"]["delay"] = total_v_data["no_waiting"] * self.step_length

        self.curr_step += 1

        if self.track_juncs:
            for junc_id, junc in self.tracked_juncs.items():
                junc.update()
                data["junctions"][junc_id] = junc.get_curr_data()

        return data, all_v_data
    
    def set_phases(self, junc_phases, start_phase = 0) -> None:
        """
        Sets the phases for the simulation.
        :param junc_phases: Junction phases and times dictionary
        :param start_phase: Phase number to start at, defaults to 0
        """

        # junc_phases = {junction_0: {phases: [phase_0, phase_1], times: [time_0, time_1]}, ...}
        # where phases are light strings and times are phase length
        self.junc_phases = junc_phases

        for junc_phase in self.junc_phases.values():

            for t in junc_phase["times"]:
                if t <= self.step_length: raise ValueError("Simulation.set_phases: Invalid phase duration (phase_dur ({0}) <= resolution ({1}))".format(t, self.step_length))

            if "curr_phase" not in junc_phase.keys(): junc_phase["curr_phase"] = start_phase
            if junc_phase["curr_phase"] > len(junc_phase["phases"]): junc_phase["curr_phase"] -= len(junc_phase["phases"])

            junc_phase["curr_time"] = sum(junc_phase["times"][:junc_phase["curr_phase"]])
            junc_phase["cycle_len"] = sum(junc_phase["times"])

    def set_phase(self, junction, phase_no) -> None:
        """
        Change to a different phase at the specified junction.
        :param junction_id: Junction ID
        :param phase_no: Phase number
        """
        
        if 0 < phase_no < len(self.junc_phases[junction]["phases"]):
            self.junc_phases[junction]["curr_phase"] = phase_no
            self.junc_phases[junction]["curr_time"] = sum(self.junc_phase["times"][:phase_no])

            self.update_lights(junction)

        else: raise ValueError("Simulation.set_phase: Invalid phase number '{0}' (must be [0-{1}])".format(phase_no, len(self.junc_phases[junction]["phases"])))

    def set_lights(self, junction_id, light_str) -> None:
        """
        Set lights to a specific setting. Will be overwritten at next light update.
        :param junction_id: Junction ID
        :param light_str: SUMO light string
        """
        traci.trafficlight.setRedYellowGreenState(junction_id, light_str)

    def update_lights(self, junction_ids = None) -> None:
        """
        Update light settings for given junctions.
        :param junction_ids: Junction ID, or list of IDs (defaults to all)
        """

        if junction_ids is None: junction_ids = self.junc_phases.keys()
        elif isinstance(junction_ids, str): junction_ids = [junction_ids]

        for junction_id in junction_ids:
            curr_setting = traci.trafficlight.getRedYellowGreenState(junction_id)
            self.light_changes += 1
            new_phase = self.junc_phases[junction_id]["phases"][self.junc_phases[junction_id]["curr_phase"]].lower()
            if '-' in new_phase:
                new_phase = new_phase.split()
                new_phase = "".join([new_phase[i] if new_phase[i] != '-' else curr_setting[i] for i in range(len(new_phase))])
            traci.trafficlight.setRedYellowGreenState(junction_id, new_phase)

    def vehicle_exists(self, vehicle_id):
        """
        Tests if vehicle exists in the network.
        :return bool: True if ID in list of current vehicle IDs 
        """
        return vehicle_id in self.all_curr_vehicle_ids

    def get_last_step_vehicles(self, detector_ids = None, flatten = False) -> dict:
        """
        Get the IDs of vehicles that passed over the specified detectors.
        :param detector_ids: detector ID or list of detector IDs (defaults to all)
        :param flatten: If true, all IDs are returned in a 1D array, else a dict with vehicles for each detector
        :return dict|list: Dict or list containing all vehicle IDs
        """

        detector_ids = list(self.available_detectors.keys()) if detector_ids == None else detector_ids
        detector_ids = [detector_ids] if not isinstance(detector_ids, list) else detector_ids

        vehicle_ids = [] if flatten else {}
        for detector_id in detector_ids:
            
            if detector_id not in self.available_detectors.keys(): raise KeyError("Simulation.get_last_step_vehicles: Unrecognised detector ID found ('{0}'}".format(detector_id))
            detector_type = self.available_detectors[detector_id]

            if detector_type == "inductionloop":
                detected_vehicles = list(traci.inductionloop.getLastStepVehicleIDs(detector_id))
            elif detector_type == "multientryexit":
                detected_vehicles = list(traci.multientryexit.getLastStepVehicleIDs(detector_id))
            else:
                warn("Warning: Unknown detector type '"+detector_type+"'")

            if flatten: vehicle_ids += detected_vehicles
            else: vehicle_ids[detector_id] = detected_vehicles

        if flatten: vehicle_ids = list(set(vehicle_ids)) # Assert only unique IDs. Why IDs found at multiple detectors?

        return vehicle_ids
    
    def get_vehicle_data(self, vehicle_id, vehicle_type = None):
        """
        Get data for specified vehicle, updating known_vehicles dict.
        :param vehicle_id: vehicle ID
        :param vehicle_type: Vehicle type if known
        :return dict: Vehicle data dictionary, returns None if does not exist in simulation
        """

        if vehicle_id not in self.all_curr_vehicle_ids: raise KeyError("Simulation.get_last_step_vehicles: Unrecognised vehicle ID found ('{0}')".format(vehicle_id))

        for vtype in self.all_vtypes:
            if vehicle_id.startswith(vtype):
                vehicle_type = vtype
                break

        speed = traci.vehicle.getSpeed(vehicle_id)
        lon, lat, alt = traci.vehicle.getPosition3D(vehicle_id)
        heading, ts = traci.vehicle.getAngle(vehicle_id), traci.vehicle.getDeparture(vehicle_id)

        if vehicle_id not in self.known_vehicles.keys():
            self.known_vehicles[vehicle_id] = {"type":      vehicle_type,
                                               "longitude": lon,
                                               "latitude":  lat,
                                               "speed":     speed,
                                               "length":    traci.vehicle.getLength(vehicle_id),
                                               "heading":   heading,
                                               "ts":        ts,
                                               "altitude":  alt,
                                               "last_seen": self.curr_step
                                              }
        else:
            self.known_vehicles[vehicle_id]["speed"]     = speed
            self.known_vehicles[vehicle_id]["longitude"] = lon
            self.known_vehicles[vehicle_id]["latitude"]  = lat
            self.known_vehicles[vehicle_id]["heading"]   = heading
            self.known_vehicles[vehicle_id]["ts"]        = ts
            self.known_vehicles[vehicle_id]["altitude"]  = alt
            self.known_vehicles[vehicle_id]["last_seen"] = self.curr_step

        vehicle_data = copy(self.known_vehicles[vehicle_id])
        del vehicle_data['last_seen']

        return vehicle_data

    def get_all_vehicle_data(self, types = None, assert_all_vtypes=False) -> dict:
        """
        Collects vehicle data from SUMO, by id and/or type (defaults to all vehicles).
        :param types: Type(s) of vehicles to fetch
        :param assert_all_vtypes: If true, raise error if vType not in all_vtypes found, else add to all_vtypes
        :return dict: Vehicle data by id
        """

        all_vehicle_data = {}
        total_vehicle_data = {"no_vehicles": 0, "no_waiting": 0}

        for vehicle_id in self.all_curr_vehicle_ids:

            # Saving known vehicles reduces calls to traci by not
            # fetching already known (& unchanging!) data
            if vehicle_id in self.known_vehicles.keys(): vehicle_type = self.known_vehicles[vehicle_id]["type"]
            else:

                vehicle_type = None
                for vtype in self.all_vtypes:
                    if vehicle_id.startswith(vtype):
                        vehicle_type = vtype
                        break

                if vehicle_type == None:
                    if assert_all_vtypes: raise AssertionError("Simulation.get_all_vehicle_data: Unrecognised vType for vehicle '{0}' found".format(vehicle_id))

                    vehicle_type = traci.vehicle.getVehicleClass(vehicle_id)
                    self.all_vtypes.add(vehicle_type)

            if types is None or (isinstance(types, list) and vehicle_type in types) or (isinstance(types, str) and vehicle_type == types):

                if self.get_individual_vehicle_data: vehicle_data = self.get_vehicle_data(vehicle_id, vehicle_type)
                
                all_vehicle_data[vehicle_id] = vehicle_data
                    
                total_vehicle_data["no_vehicles"] += 1
                if vehicle_data["speed"] < 0.1: total_vehicle_data["no_waiting"] += 1

        return total_vehicle_data, all_vehicle_data
    
    def get_vehicle_route(self, vehicle_id, get_edges = True):
        """
        Returns vehicle route, either route ID or array of edges.
        :param vehicle_id: Vehicle ID
        :param get_edges:  Denotes if to return list of edges or route ID
        :return [str]|str: List of edges or route ID
        """

        if get_edges: return list(traci.vehicle.getRoute(vehicle_id))
        else: return traci.vehicle.getRouteID(vehicle_id)

class Junction:
    def __init__(self, junc_id, sim, junc_params=None):
        self.junction_id = junc_id
        self.position = traci.junction.getPosition(junc_id)

        self.sim = sim
        self.sim_step_length = sim.step_length
        self.init_time = sim.curr_step * sim.step_length
        self.curr_time = sim.curr_step * sim.step_length
        
        self.has_tl = junc_id in sim.all_tls

        if self.has_tl:
            state_str = traci.trafficlight.getRedYellowGreenState(junc_id)
            self.m_len = len(state_str)
            self.states, self.curr_state = [], []        
            self.durations = [[] for _ in range(self.m_len)]
            self.avg_green, self.avg_m_red = 0, 0
            self.avg_m_green, self.avg_m_red = [0 for _ in range(self.m_len)], [0 for _ in range(self.m_len)]

        if junc_params != None and "inflow_detectors" in junc_params.keys() and "outflow_detectors" in junc_params.keys():
            self.inflow_detectors = junc_params["inflow_detectors"]
            self.outflow_detectors = junc_params["outflow_detectors"]

            if "flow_vtypes" in junc_params.keys(): self.flow_vtypes = ["all"] + junc_params["flow_vtypes"]
            else: self.flow_vtypes = ["all"]
            
            self.v_in, self.v_out = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}
            self.avg_inflow, self.avg_outflow = {vtype: 0 for vtype in self.flow_vtypes}, {vtype: 0 for vtype in self.flow_vtypes}
            self.inflows, self.outflows = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}

            self.track_flow = True

        else: self.track_flow = False

        self.update()
    
    def __str__(self): return "<Junction: '{0}'>".format(self.junction_id)

    def get_curr_data(self):
        """
        Returns the current state of junction data as a dictionary.
        :return dict: Junction data dictionary
        """

        junc_dict = {"position": self.position, "init_time": self.init_time, "curr_time": self.curr_time}
        
        if self.has_tl: junc_dict["tl"] = {"m_len": self.m_len, "avg_green": self.avg_green, "avg_red": self.avg_red,
                                           "avg_m_green": self.avg_m_green, "avg_m_red": self.avg_m_red, "m_phases": self.durations}

        if self.track_flow:
            junc_dict["flows"] = {"inflow_detectors": self.inflow_detectors, "outflow_detectors": self.outflow_detectors,
                                 "avg_inflows": self.avg_inflow, "avg_outflows": self.avg_outflow,
                                 "all_inflows": self.inflows, "all_outflows": self.outflows}

        return junc_dict

    def reset(self):
        """
        Resets junction data collection.
        """
        
        self.init_time = self.curr_time = self.sim.curr_step * self.sim_step_length
        self.curr_time = self.curr_time = self.sim.curr_step * self.sim_step_length

        if self.has_tl:
            self.states = []
            self.durations = [[] for _ in range(self.m_len)]
            self.avg_green, self.avg_m_red = 0, 0
            self.avg_m_green, self.avg_m_red = [0 for _ in range(self.m_len)], [0 for _ in range(self.m_len)]

        if self.track_flow:
            self.v_in, self.v_out = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}
            self.avg_inflow, self.avg_outflow = {vtype: 0 for vtype in self.flow_vtypes}, {vtype: 0 for vtype in self.flow_vtypes}
            self.inflows, self.outflows = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}

    def update(self):
        """
        Update junction flow and TL data for the current time step.
        """

        self.curr_time = self.sim.curr_step * self.sim_step_length
        
        if self.has_tl:
            curr_state = traci.trafficlight.getRedYellowGreenState(self.junction_id)
            colours = [*curr_state]
            for idx, mc in enumerate(colours):
                
                if len(self.durations[idx]) == 0 or mc.upper() != self.durations[idx][-1][0]:
                    self.durations[idx].append([mc.upper(), self.sim_step_length])
                elif mc.upper() == self.durations[idx][-1][0]:
                    self.durations[idx][-1][1] = self.durations[idx][-1][1] + self.sim_step_length

                if mc.upper() == 'G':
                    m_green_durs = [val[1] for val in self.durations[idx] if val[0] == 'G']
                    self.avg_m_green[idx] = sum(m_green_durs) / len(m_green_durs)
                elif mc.upper() == 'R':
                    m_red_durs = [val[1] for val in self.durations[idx] if val[0] == 'R']
                    self.avg_m_red[idx] = sum(m_red_durs) / len(m_red_durs)

            self.avg_green = sum(self.avg_m_green) / len(self.avg_m_green)
            self.avg_red = sum(self.avg_m_red) / len(self.avg_m_red)

            self.states.append(colours)
            self.curr_state = colours

        if self.track_flow:
            step_v_in = self.sim.get_last_step_vehicles(self.inflow_detectors, flatten=True)
            step_v_out = self.sim.get_last_step_vehicles(self.outflow_detectors, flatten=True)

            for vtype in self.flow_vtypes:
                new_v_in = [v_id for v_id in step_v_in if v_id not in self.v_in[vtype] and (vtype == "all" or v_id.startswith(vtype))]
                self.v_in[vtype] += new_v_in
                self.inflows[vtype].append(len(new_v_in))
                self.avg_inflow[vtype] = (sum(self.inflows[vtype]) / len(self.inflows[vtype])) / self.sim_step_length

            for vtype in self.flow_vtypes:
                new_v_out = [v_id for v_id in step_v_out if v_id not in self.v_out[vtype] and (vtype == "all" or v_id.startswith(vtype))]
                self.v_out[vtype] += new_v_out
                self.outflows[vtype].append(len(new_v_out))
                self.avg_outflow[vtype] = (sum(self.outflows[vtype]) / len(self.outflows[vtype])) / self.sim_step_length

def get_cumulative_arr(arr, start) -> list:
    for i in range(start, len(arr)):
        arr[i] += arr[i - 1]
    return arr