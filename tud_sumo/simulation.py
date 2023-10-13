import os, sys, traci, json, math
from tqdm import tqdm
from copy import copy, deepcopy
from events import EventScheduler
from enum import Enum

class Simulation:
    def __init__(self, all_vtypes=[]) -> None:
        path_tools = os.path.join(os.environ.get("SUMO_HOME"), 'tools')

        if path_tools in sys.path: pass
        else: sys.path.append(path_tools)

        self.running = False
        self.curr_step = 0
        self.gui = False

        self.junc_phases = None

        self.track_juncs = False
        self.all_juncs = []
        self.tracked_juncs = {}

        self.all_edges = None
        self.all_lanes = None

        self.available_detectors = {}
        self.step_length = None

        self.get_individual_vehicle_data = True
        self.all_curr_vehicle_ids = []
        self.known_vehicles = {}
        self.light_changes = 0

        self.all_data = None

        self.all_vtypes = set(all_vtypes)

        self.scheduler = None

    def start(self, config_file = None, net_file = None, route_file = None, add_file = None, cmd_options = None, units=1, suppress_warnings = False, ignore_TraCI_err = True, gui = False) -> None:
        """
        Intialises SUMO simulation.
        :param config_file: Location of '.sumocfg' file (can be given instead of net_file)
        :param net_file:    Location of '.net.xml' file (can be given instead of config_file)
        :param route_file:  Location of '.rou.xml' route file
        :param add_file:    Location of '.add.xml' additional file
        :param cmd_options: List of any other command line options
        :param units:       Data collection units [1 (metric) | 2 (IMPERIAL) | 3 (UK)] (defaults to 'metric')
        :param suppress_warnings: Suppress simulation warnings
        :param ignore_TraCI_err: If true and a fatal traCI error occurs, the simulation ends but the program continues to run
        :param gui:         Bool denoting whether to run GUI
        """

        self.gui = gui
        sumoCMD = ["sumo-gui"] if gui else ["sumo"]

        if config_file == net_file == None: raise ValueError("Sim.start: Either config or network file required.")
        
        if config_file != None:
            if config_file.endswith(".sumocfg"): sumoCMD += ["-c", config_file]
            else: raise ValueError("Sim.start: Invalid config file extension.")
        else:
            sumoCMD += ["-n", net_file]
            if route_file != None: sumoCMD += ["-r", route_file]
            if add_file != None: sumoCMD += ["-a", add_file]
            if cmd_options != None: sumoCMD += cmd_options

        if units in [1, 2, 3]: self.units = Units(units)
        else: raise ValueError("Plot.init: Invalid simulation units '{0}' (must be 1-3)".format(units))

        traci.start(sumoCMD)
        self.running = True

        self.step_length = float(traci.simulation.getOption("step-length"))

        if self.junc_phases != None: self.update_lights()
        self.time_val = traci.simulation.getTime()

        self.available_detectors = {detector_id: 'multientryexit' for detector_id in list(traci.multientryexit.getIDList())}
        self.available_detectors.update({detector_id: 'inductionloop' for detector_id in list(traci.inductionloop.getIDList())})

        self.all_edges = traci.edge.getIDList()
        self.all_lanes = traci.lane.getIDList()

        self.ignore_TraCI_err = ignore_TraCI_err
        self.suppress_warnings = suppress_warnings
    
    def toggle_vehicle_tracking(self):
        """
        Toggles the collection of individual vehicle data.
        :param bool: Denotes whether data is being collected
        """

        self.get_individual_vehicle_data = not self.get_individual_vehicle_data
        return self.get_individual_vehicle_data
        
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
            else: raise TypeError("Sim.start_junc_tracking: Invalid junc_params (must be [str|list|dict], not '"+type(juncs).__name__+"').")

            if len(set(all_juncs).intersection(set(junc_ids))) != len(junc_ids):
                raise KeyError("Sim.start_junc_tracking: Junction ID not found.")
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
        """
        Returns whether the simulation is running.
        :param close: If True, end Simulation
        :return bool: Denotes if the simulation is running
        """
        if not self.running: return self.running

        elif traci.simulation.getMinExpectedNumber() == 0:
            if close: self.end()
            print('Ended simulation: no vehicles remaining.')
            return False
        
        return True

    def end(self):
        """
        Ends the simulation.
        """

        traci.close()
        self.running = False

    def save_data(self, filename, overwrite=True):
        """
        Save all vehicle, detector and junction data in a JSON file.
        :param filename:  Output filepath
        :param overwrite: Prevent previous outputs being overwritten
        """
        if os.path.exists(filename) and overwrite:
            if not self.suppress_warnings: print("(WARNING) Sim.save_data: Simulation.save_data: File '{0}' already exists and will be overwritten".format(filename))
        elif os.path.exists(filename) and not overwrite:
            raise FileExistsError("Sim.save_data: File '{0}' already exists and can't be overwritten.".format(filename))
        
        if not filename.endswith(".json"): filename += ".json"

        if self.all_data != None:
            if self.scheduler != None: self.all_data["data"]["events"] = self.scheduler.__dict__()
            with open(filename, "w") as fp:
                json.dump(self.all_data, fp, indent=4)
        else: raise AssertionError("Sim.save_data: No data to save as simulation has not been run.")

    def add_events(self, event_params):
        """
        Add events and event scheduler.
        :param event_parms: Event parameters [Event|[Event]|dict|filepath]
        """
        if self.scheduler == None:
            self.scheduler = EventScheduler(self)
        self.scheduler.add_events(event_params)

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
        elif end_step == None: raise ValueError("Sim.step_through: No time value given.")

        if prev_data == None:
            prev_steps, all_data = 0, {"data": {"detector": {}, "junctions": {}, "vehicle": {}, "all_vehicles": []},
                                       "step_len": self.step_length, "units": self.units.name, "start": start_time}
            if self.scheduler != None: all_data["data"]["events"] = {}
        else: 
            prev_steps = set([len(data_arr) for data_arr in prev_data["data"]["vehicle"].values()] +
                            [len(detector_data["speeds"]) for detector_data in prev_data["data"]["detector"].values()] +
                            [len(detector_data["veh_counts"]) for detector_data in prev_data["data"]["detector"].values()])
            
            if len(prev_steps) != 1: raise ValueError("Sim.step_through: Invalid prev_data (different length arrays).")
            else:
                prev_steps = prev_steps.pop()
                all_data = prev_data

        if not self.gui and n_steps > 1: pbar = tqdm(desc="Running simulation", total=end_step - self.curr_step)
        while self.curr_step < end_step:

            try: last_step_data, all_v_data = self.step(detector_list, vTypes)
            except traci.exceptions.FatalTraCIError:
                self.running = False
                if self.ignore_TraCI_err:
                    if not self.suppress_warnings: print("(WARNING) Sim.step_through: Fatal TraCI connection error occured and 'Sim.ignore_TraCI_err' is set to True.")
                    break
                else: raise traci.exceptions.FatalTraCIError("Sim.step_through: Fatal TraCI connection error occured.")

            all_data["data"]["all_vehicles"].append(all_v_data)
            all_data["data"]["junctions"] = last_step_data["junctions"]

            if len(all_data["data"]["detector"]) == 0:
                for detector_id in detector_list:
                    all_data["data"]["detector"][detector_id] = {"type": self.available_detectors[detector_id], "speeds": [], "veh_counts": [], "veh_ids": [], "occupancies": []}
                all_data["data"]["vehicle"] = {"no_vehicles": [], "tts": [], "delay": []}

            for detector_id in last_step_data["detector"].keys():
                if detector_id not in all_data["data"]["detector"].keys(): raise KeyError("Sim.step_through: Unrecognised detector ID found ('{0}').".format(detector_id))
                for data_key, data_val in last_step_data["detector"][detector_id].items():
                    all_data["data"]["detector"][detector_id][data_key].append(data_val)

            for data_key, data_val in last_step_data["vehicle"].items():
                all_data["data"]["vehicle"][data_key].append(data_val)

            if not self.gui and n_steps > 1: pbar.update(1)

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

                # Change to a while loop, updating phase until correct is found? Would then allow for phases with dur less than the step len
                elif phases["curr_time"] >= sum(phases["times"][:phases["curr_phase"] + 1]):
                    phases["curr_phase"] += 1
                    update_junc_lights.append(junction_id)

            self.update_lights(update_junc_lights)

        if self.scheduler != None: self.scheduler.update_events()

        if detector_list == None: detector_list = list(self.available_detectors.keys())
        for detector_id in detector_list:
            data["detector"][detector_id] = {}
            if detector_id not in self.available_detectors.keys(): raise KeyError("Sim.step_through: Unrecognised detector ID found ('{0}').".format(detector_id))
            if self.available_detectors[detector_id] == "multientryexit":

                speed = traci.multientryexit.getLastStepMeanSpeed(detector_id)
                if self.units.name in ['IMPERIAL', 'UK']: data["detector"][detector_id]["speeds"] = speed * 2.2369362920544
                else: data["detector"][detector_id]["speeds"] = speed * 3.6

                data["detector"][detector_id]["veh_counts"] = traci.multientryexit.getLastStepVehicleNumber(detector_id)
                
            elif self.available_detectors[detector_id] == "inductionloop":

                speed = traci.multientryexit.inductionloop(detector_id)
                if self.units.name in ['IMPERIAL', 'UK']: data["detector"][detector_id]["speeds"] = speed * 2.2369362920544
                else: data["detector"][detector_id]["speeds"] = speed * 3.6

                data["detector"][detector_id]["veh_counts"] = traci.inductionloop.getLastStepVehicleNumber(detector_id)
                data["detector"][detector_id]["occupancies"] = traci.inductionloop.getLastStepOccupancy(detector_id)

            else:
                if not self.suppress_warnings: print("(WARNING) Sim.step: Unknown detector type '"+self.available_detectors[detector_id]+"'")

            data["detector"][detector_id]["veh_ids"] = self.get_last_step_detector_vehicles(detector_id)

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
    
    def set_phases(self, new_junc_phases, start_phase = 0, overwrite=True) -> None:
        """
        Sets the phases for the simulation.
        :param new_junc_phases: Junction phases and times dictionary
        :param start_phase: Phase number to start at, defaults to 0
        :param overwrite: If true, the junc_phases dict is overwitten with new_junc_phases. If false, only specific junctions are overwritten.
        """

        # junc_phases = {junction_0: {phases: [phase_0, phase_1], times: [time_0, time_1]}, ...}
        # where phases are light strings and times are phase length

        if overwrite or self.junc_phases == None:
            self.junc_phases = new_junc_phases
        else:
            for junc_id, new_phases in new_junc_phases.items():
                self.junc_phases[junc_id] = deepcopy(new_phases)

        for junc_id in new_junc_phases.keys():

            if junc_id not in list(traci.trafficlight.getIDList()):
                raise KeyError("Sim.set_phases: Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(junc_id))

            junc_phase = self.junc_phases[junc_id]

            for t in junc_phase["times"]:
                if t <= self.step_length: raise ValueError("Sim.set_phases: Invalid phase duration (phase_dur ({0}) <= resolution ({1})).".format(t, self.step_length))

            if "curr_phase" not in junc_phase.keys(): junc_phase["curr_phase"] = start_phase
            if junc_phase["curr_phase"] > len(junc_phase["phases"]): junc_phase["curr_phase"] -= len(junc_phase["phases"])

            junc_phase["curr_time"] = sum(junc_phase["times"][:junc_phase["curr_phase"]])
            junc_phase["cycle_len"] = sum(junc_phase["times"])

        self.update_lights(new_junc_phases.keys())

    def set_tl_colour(self, junction_id, colour_str) -> None:
        """
        Sets a junction to a set colour/phase string for an indefinite amount of time. Can be used when tracking phases separately.
        :param junction_id: Junction ID
        :param colour_str: Colour character ([r|y|g|-]) or phase string ([r|y|g|-]+)
        """
        
        if junction_id not in list(traci.trafficlight.getIDList()):
            raise KeyError("Sim.set_tl_colour: Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(junction_id))
        else:
            if junction_id in self.tracked_juncs.keys():
                m_len = self.tracked_juncs[junction_id].m_len
            else:
                state_str = traci.trafficlight.getRedYellowGreenState(junction_id)
                m_len = len(state_str)

        if not isinstance(colour_str, str): raise TypeError("Sim.set_tl_colour: Invalid colour_str (must be 'str', not '{0}').".format(type(colour_str).__name__))
        
        if len(colour_str) == 1:
            junc_phases = {junction_id: {"phases": [colour_str*m_len], "times": [math.inf]}}
        elif len(colour_str) == m_len:
            junc_phases = {junction_id: {"phases": [colour_str], "times": [math.inf]}}
        else:
            raise ValueError("Sim.set_tl_colour: Invalid colour_str (must be char or len(str) = junction movements length).")
        
        self.set_phases(junc_phases, overwrite=False)

    def set_tl_metering_rate(self, junction_id, flow_rate, min_red, y_dur, green_time=1):
        """
        Set ramp metering rate by flow at a junction. Uses a one-car-per-green policy, with a default 1s green phase duration.
        :param junction_id: Junction ID
        :param flow_rate: On-ramp inflow (veh/hr)
        :param min_red: Minimum red duration (s)
        :param y_dur: Yellow phase duration (s)
        :param green_time: Green phase duration, defaults to 1s (s)
        :return dict: Phase dictionary
        """

        if junction_id not in list(traci.trafficlight.getIDList()):
            raise KeyError("Sim.set_tl_colour: Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(junction_id))
        else:
            if junction_id in self.tracked_juncs.keys():
                m_len = self.tracked_juncs[junction_id].m_len
            else:
                state_str = traci.trafficlight.getRedYellowGreenState(junction_id)
                m_len = len(state_str)

        flow_pm = flow_rate / 60
        cycle_length = 60 / flow_pm

        red_time = cycle_length - green_time - y_dur
        if red_time < min_red or red_time <= 0:
            phases_dict = {"phases": ['G'*m_len], "times": [cycle_length]}

        else:
            phases_dict = {"phases": ['G'*m_len, 'y'*m_len, 'r'*m_len],
                           "times":  [green_time / self.step_length, y_dur / self.step_length, red_time / self.step_length]}
              
        self.set_phases({junction_id: phases_dict}, overwrite=False)
        
        return phases_dict

    def change_phase(self, junction_id, phase_no) -> None:
        """
        Change to a different phase at the specified junction_id.
        :param junction_id: Junction ID
        :param phase_no: Phase number
        """
        
        if 0 < phase_no < len(self.junc_phases[junction_id]["phases"]):
            self.junc_phases[junction_id]["curr_phase"] = phase_no
            self.junc_phases[junction_id]["curr_time"] = sum(self.junc_phase["times"][:phase_no])

            self.update_lights(junction_id)

        else: raise ValueError("Sim.change_phase: Invalid phase number '{0}' (must be [0-{1}]).".format(phase_no, len(self.junc_phases[junction_id]["phases"])))

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

    def get_last_step_detector_vehicles(self, detector_ids, v_types = None, flatten = False) -> dict|list:
        """
        Get the IDs of vehicles that passed over the specified detectors.
        :param detector_ids: detector ID or list of detector IDs (defaults to all)
        :param v_types: Included vehicle types
        :param flatten: If true, all IDs are returned in a 1D array, else a dict with vehicles for each detector
        :return dict|list: Dict or list containing all vehicle IDs
        """

        detector_ids = [detector_ids] if not isinstance(detector_ids, list) else detector_ids
        if len(detector_ids) == 1: flatten = True
        v_types = [v_types] if v_types != None and not isinstance(v_types, list) else v_types

        vehicle_ids = [] if flatten else {}
        for detector_id in detector_ids:
            
            if detector_id not in self.available_detectors.keys(): raise KeyError("Sim.get_last_step_detector_vehicles: Detector ID '{0}' not found.".format(detector_id))
            detector_type = self.available_detectors[detector_id]

            if detector_type == "inductionloop":
                detected_vehicles = list(traci.inductionloop.getLastStepVehicleIDs(detector_id))
            elif detector_type == "multientryexit":
                detected_vehicles = list(traci.multientryexit.getLastStepVehicleIDs(detector_id))
            else:
                raise KeyError("Sim.get_last_step_detector_vehicles: Unknown detector type '{0}'".format(detector_type))
            
            if v_types != None:
                detected_vehicles = [vehicle_id for vehicle_id in detected_vehicles if self.get_vehicle_vals(vehicle_id, "type") in v_types]

            if flatten: vehicle_ids += detected_vehicles
            else: vehicle_ids[detector_id] = detected_vehicles

        if flatten: vehicle_ids = list(set(vehicle_ids))

        return vehicle_ids
    
    def set_vehicle_vals(self, vehicle_id, **kwargs):
        """
        Calls the TraCI API to change a vehicle's state.
        :param vehicle_id: Vehicle ID
        :param highlight: Highlights vehicle with specified colour [str|(int)]
        :param setSpeed: Set new speed value [int|float]
        :param setMaxSpeed: Set new max speed value [int|float]
        :param setAcceleration: Set acceleration for a given duration ([int|float], [int|float])
        :param changeLane: Try and change lane for a given duration (int, [int|float])
        :param changeTarget: Set vehicle destination edge ID [str]
        :param changeRoute: Set vehicle route, either by route ID or list of edges [str|(str)]
        """
        
        if not self.vehicle_exists(vehicle_id):
            raise KeyError("Sim.set_vehicle_vals: Unrecognised vehicle ID given ('{0}').".format(vehicle_id))
        
        for command, value in kwargs.items():
            match command:
                case "highlight":
                    if value != None:
                        colour = value
                        if isinstance(colour, str):
                            if "#" in colour: colour = colour.lstrip("#")
                            if len(colour) != 6: raise ValueError("Sim.set_vehicle_vals ({0}): '{1}' is not a valid hex colour.".format(command, colour))
                            colour = tuple(int(colour[i:i+2], 16) for i in (0, 2, 4))
                        elif not (isinstance(colour, list) or isinstance(colour, tuple)):
                            raise TypeError("Sim.set_vehicle_vals ({0}): Invalid colour (must be [str|list|tuple], not '{1}').".format(command, type(colour).__name__))
                        elif len(colour) not in [3, 4] or all(x > 255 for x in colour) or all(x < 0 for x in colour):
                            raise ValueError("Sim.set_vehicle_vals ({0}): '{1}' is not a valid RGB or RGBA colour.".format(command, colour))
                        
                        if len(colour) == 3: colour = list(colour) + [255]

                        traci.vehicle.setColor(vehicle_id, colour)
                    else:
                        v_type = self.get_vehicle_vals(vehicle_id, "type")
                        type_colour = tuple(traci.vehicletype.getColor(v_type))
                        traci.vehicle.setColor(vehicle_id, type_colour)

                case "speed":
                    if isinstance(value, int) or isinstance(value, float): 
                        if self.units.name in ['IMPERIAL', 'UK']: traci.vehicle.setSpeed(vehicle_id, value / 2.2369362920544)
                        else: traci.vehicle.setSpeed(vehicle_id, value / 3.6)
                    else: raise TypeError("Sim.set_vehicle_vals ({0}): Invalid speed value '{1}' (must be [int|float], not '{2}').".format(command, value, type(value).__name__))
                
                case "max_speed":
                    if isinstance(value, int) or isinstance(value, float):
                        if self.units.name in ['IMPERIAL', 'UK']: traci.vehicle.setMaxSpeed(vehicle_id, value / 2.2369362920544)
                        else: traci.vehicle.setMaxSpeed(vehicle_id, value / 3.6)
                    else: raise TypeError("Sim.set_vehicle_vals ({0}): Invalid max speed value '{1}' (must be [int|float], not '{2}').".format(command, value, type(value).__name__))
                
                case "acceleration":
                    if hasattr(value, "__iter__") and len(value) == 2:
                        if not (isinstance(value[0], int) or isinstance(value[0], float)):
                            raise TypeError("Sim.set_vehicle_vals ({0}): Invalid acceleration '{1}' (must be [int|float], not '{2}').".format(command, value[0], type(value[0]).__name__))
                        if not (isinstance(value[1], int) or isinstance(value[1], float)):
                            raise TypeError("Sim.set_vehicle_vals ({0}): Invalid laneIndex '{1}' (must be [int|float], not '{2}').".format(command, value[1], type(value[1]).__name__))
                        traci.vehicle.setAcceleration(vehicle_id, float(value[0]), float(value[1]))
                    else: raise TypeError("Sim.set_vehicle_vals ({0}): '{0}' requires 2 parameters (acceleration [int|float], duration [int|float])".format(command))

                case "lane_idx":
                    if hasattr(value, "__iter__") and len(value) == 2:
                        if not isinstance(value[0], int): raise TypeError("Sim.set_vehicle_vals ({0}): Invalid laneIndex '{1}' (must be int, not '{2}').".format(command, value[0], type(value[0]).__name__))
                        if not (isinstance(value[1], int) or isinstance(value[1], float)):
                            raise TypeError("Sim.set_vehicle_vals ({0}): Invalid duration '{1}' (must be [int|float], not '{2}').".format(command, value[1], type(value[1]).__name__))
                        traci.vehicle.changeLane(vehicle_id, value[0], float(value[1]))
                    else: raise TypeError("Sim.set_vehicle_vals ({0}): '{0}' requires 2 parameters (laneIndex [int], duration [int|float])".format(command))

                case "target":
                    if isinstance(value, str): traci.vehicle.changeTarget(vehicle_id, value)
                    else: raise TypeError("Sim.set_vehicle_vals ({0}): Invalid edge ID '{1}' (must be str, not '{2}').".format(command, value, type(value).__name__))
                
                case "route_id":
                    if isinstance(value, str): traci.vehicle.setRouteID(vehicle_id, value)
                    else: raise TypeError("Sim.set_vehicle_vals ({0}): Invalid route ID value '{1}' (must be str, not '{2}').".format(command, value, type(value).__name__))
                
                case "route_edges":
                    if hasattr(value, "__iter__") and all(isinstance(x, str) for x in value): traci.vehicle.setRoute(value)
                    else: raise TypeError("Sim.set_vehicle_vals ({0}): Invalid route egdes value '{1}' (must be (str), not '{2}').".format(command, value, type(value).__name__))

                case _:
                    raise ValueError("Sim.set_vehicle_vals: Unrecognised command ('{0}').".format(command))

    def get_vehicle_vals(self, vehicle_id, data_keys):
        """
        Get data values for specific vehicle using a list of data keys.
        :param vehicle_id: Vehicle ID
        :param data_keys: List of keys from [len|speed|pos|heading|ts|route_id|route_edges], or single key
        :return dict: Values by data_key
        """

        if not self.vehicle_exists(vehicle_id):
            raise KeyError("Sim.get_vehicle_vals: Unrecognised vehicle ID given ('{0}').".format(vehicle_id))

        return_val = False
        if isinstance(data_keys, str):
            data_keys = [data_keys]
            return_val = True
        elif not hasattr(data_keys, "__iter__"):
            raise TypeError("Sim.get_vehicle_vals: Invalid data_keys given '{1}' (must be [str|(str)], not '{2}').".format(data_keys, type(data_keys).__name__))
        
        data_vals = {}
        for data_key in data_keys:
            match data_key:
                case "type":
                    if vehicle_id not in self.known_vehicles.keys():
                        data_vals[data_key] = traci.vehicle.getTypeID(vehicle_id)
                    else: data_vals[data_key] = self.known_vehicles[vehicle_id]["type"]
                case "length":
                    if vehicle_id not in self.known_vehicles.keys():
                        length = traci.vehicle.getLength(vehicle_id)
                        if self.units.name == 'IMPERIAL': length *= 3.28084
                        data_vals[data_key] = length
                    else: data_vals[data_key] = self.known_vehicles[vehicle_id]["length"]
                case "speed":
                    speed = traci.vehicle.getSpeed(vehicle_id)
                    if self.units.name in ['IMPERIAL', 'UK']: speed *= 2.2369362920544
                    else: speed *= 3.6
                    data_vals[data_key] = speed
                case "max_speed":
                    max_speed = traci.vehicle.getMaxSpeed(vehicle_id)
                    if self.units.name in ['IMPERIAL', 'UK']: max_speed *= 2.2369362920544
                    else: max_speed *= 3.6
                    data_vals[data_key] = max_speed
                case "acceleration":
                    data_vals[data_key] = traci.vehicle.getAcceleration(vehicle_id)
                case "position":
                    data_vals[data_key] = traci.vehicle.getPosition3D(vehicle_id)
                case "heading":
                    data_vals[data_key] = traci.vehicle.getAngle(vehicle_id)
                case "ts":
                    data_vals[data_key] = traci.vehicle.getDeparture(vehicle_id)
                case "lane_idx":
                    data_vals[data_key] = traci.vehicle.getLaneIndex(vehicle_id)
                case "target":
                    data_vals[data_key] = list(traci.vehicle.getRoute(vehicle_id))[-1]
                case "route_id":
                    data_vals[data_key] = traci.vehicle.getRouteID(vehicle_id)
                case "route_edges":
                    data_vals[data_key] = list(traci.vehicle.getRoute(vehicle_id))
                case _:
                    raise ValueError("Sim.get_vehicle_vals: Unrecognised key ('{0}').".format(data_key))

        if set(data_vals.keys()) != set(data_vals):
            raise ValueError("Sim.get_edge_vals: Invalid data_keys given (must be from accepted list).")
        if return_val: return list(data_vals.values())[0]
        else: return data_vals
    
    def get_vehicle_data(self, vehicle_id, vehicle_type = None, assert_exists=True):
        """
        Get data for specified vehicle, updating known_vehicles dict.
        :param vehicle_id: Vehicle ID
        :param vehicle_type: Vehicle type if known
        :param assert_exists: If True, error is raised if vehicle does not exist
        :return dict: Vehicle data dictionary, returns None if does not exist in simulation
        """

        if not self.vehicle_exists(vehicle_id):
            if assert_exists: raise KeyError("Sim.get_vehicle_data: Unrecognised vehicle ID found ('{0}').".format(vehicle_id))
            elif not self.suppress_warnings:
                print("(WARNING) Sim.get_vehicle_data: Unrecognised vehicle ID given ('{0}').".format(vehicle_id))
                return None
        
        vehicle_data = self.get_vehicle_vals(vehicle_id, ("type", "speed", "position", "heading", "ts", "length"))

        if vehicle_id not in self.known_vehicles.keys():
            self.known_vehicles[vehicle_id] = {"type":      vehicle_data["type"],
                                               "longitude": vehicle_data["position"][0],
                                               "latitude":  vehicle_data["position"][1],
                                               "speed":     vehicle_data["speed"],
                                               "length":    vehicle_data["length"],
                                               "heading":   vehicle_data["heading"],
                                               "ts":        vehicle_data["ts"],
                                               "altitude":  vehicle_data["position"][2],
                                               "last_seen": self.curr_step
                                              }
        else:
            self.known_vehicles[vehicle_id]["speed"]     = vehicle_data["speed"]
            self.known_vehicles[vehicle_id]["longitude"] = vehicle_data["position"][0]
            self.known_vehicles[vehicle_id]["latitude"]  = vehicle_data["position"][1]
            self.known_vehicles[vehicle_id]["heading"]   = vehicle_data["heading"]
            self.known_vehicles[vehicle_id]["ts"]        = vehicle_data["ts"]
            self.known_vehicles[vehicle_id]["altitude"]  = vehicle_data["position"][2]
            self.known_vehicles[vehicle_id]["last_seen"] = self.curr_step

        vehicle_data = copy(self.known_vehicles[vehicle_id])
        del vehicle_data['last_seen']

        return vehicle_data

    def get_all_vehicle_data(self, types = None, assert_all_vtypes=False) -> dict:
        """
        Collects vehicle data from SUMO, by id and/or type (defaults to all vehicles).
        :param types: Type(s) of vehicles to fetch
        :param assert_all_vtypes: If true, raise error if vType not in all_vtypes found, else add to all_vtypes
        :return dict: Stats & vehicle data by id
        """

        all_vehicle_data = {}
        total_vehicle_data = {"no_vehicles": 0, "no_waiting": 0}

        for vehicle_id in self.all_curr_vehicle_ids:

            # Saving known vehicles reduces calls to TraCI by not
            # fetching already known (& unchanging!) data
            if vehicle_id in self.known_vehicles.keys(): vehicle_type = self.known_vehicles[vehicle_id]["type"]
            else:

                vehicle_type = None
                for vtype in self.all_vtypes:
                    if vehicle_id.startswith(vtype):
                        vehicle_type = vtype
                        break

                if vehicle_type == None:
                    if assert_all_vtypes: raise AssertionError("Sim.get_all_vehicle_data: Unrecognised vType for vehicle '{0}' found.".format(vehicle_id))

                    vehicle_type = traci.vehicle.getTypeID(vehicle_id)
                    self.all_vtypes.add(vehicle_type)

            if types is None or (isinstance(types, list) and vehicle_type in types) or (isinstance(types, str) and vehicle_type == types):

                if self.get_individual_vehicle_data:
                    vehicle_data = self.get_vehicle_data(vehicle_id, vehicle_type)
                    all_vehicle_data[vehicle_id] = vehicle_data
                    speed = vehicle_data["speed"]
                else:
                    speed = self.get_vehicle_vals(vehicle_id, "speed")

                total_vehicle_data["no_vehicles"] += 1
                if speed < 0.1: total_vehicle_data["no_waiting"] += 1

        return total_vehicle_data, all_vehicle_data
    
    def get_geometry_vals(self, geometry_id, data_keys):
        """
        Get data values for specific edge or lane using a list of data keys, from: (edge or lane) vehicle_ids, vehicle_speed,
        vehicle_halting, vehicle_occupancy, tt (travel time), emissions, (edge only) street_name, n_lanes, (lane only) edge_id
        n_links, permissions, max_speed
        :param edge_id: Either lane or edge ID
        :param data_keys: List of keys or single key
        :return [float]: Data value
        """
        
        g_name = self.geometry_exists(geometry_id)
        if g_name == "edge": g_class = traci.edge
        elif g_name == "lane": g_class = traci.lane
        else: raise KeyError("Sim.get_edge_vals: Geometry ID '{0}' not found.".format(geometry_id))

        return_val = False
        if isinstance(data_keys, str):
            data_keys = [data_keys]
            return_val = True
        elif not hasattr(data_keys, "__iter__"):
            raise TypeError("Sim.get_edge_vals: Invalid data_keys given '{1}' (must be [str|(str)], not '{2}').".format(data_keys, type(data_keys).__name__))
        
        data_vals = {}
        for data_key in data_keys:
            match data_key:
                case "vehicle_ids":
                    data_vals[data_key] = g_class.getLastStepVehicleIDs(geometry_id)
                    continue
                case "vehicle_speed":
                    if self.units.name in ['IMPERIAL', 'UK']: data_vals[data_key] = g_class.getLastStepMeanSpeed(geometry_id) * 2.2369362920544
                    else: data_vals[data_key] = g_class.getLastStepMeanSpeed(geometry_id) * 3.6
                    continue
                case "vehicle_halting":
                    data_vals[data_key] = g_class.getLastStepHaltingNumber(geometry_id)
                    continue
                case "vehicle_occupancy":
                    data_vals[data_key] = g_class.getLastStepOccupancy(geometry_id)
                    continue
                case "tt":
                    data_vals[data_key] = g_class.getTraveltime(geometry_id)
                    continue
                case "emissions":
                    data_vals[data_key] = ({"CO2": g_class.getCO2Emission(geometry_id), "CO": g_class.getCO2Emission(geometry_id), "HC": g_class.getHCEmission(geometry_id),
                                            "PMx": g_class.getPMxEmission(geometry_id), "NOx": g_class.getNOxEmission(geometry_id)})
                    continue
                    
            if g_name == "edge":
                match data_key:
                    case "street_name":
                        data_vals[data_key] = g_class.getStreetName(geometry_id)
                    case "n_lanes":
                        data_vals[data_key] = g_class.getLaneNumber(geometry_id)
                    case _:
                        raise ValueError("Sim.get_vehicle_vals: Unrecognised key ('{0}').".format(data_key))
            elif g_name == "lane":
                match data_key:
                    case "edge_id":
                        data_vals[data_key] = g_class.getEdgeID(geometry_id)
                    case "n_links":
                        data_vals[data_key] = g_class.getLinkNumber(geometry_id)
                    case "allowed":
                        data_vals[data_key] = g_class.getAllowed(geometry_id)
                    case "disallowed":
                        data_vals[data_key] = g_class.getDisallowed(geometry_id)
                    case "left_lc":
                        data_vals[data_key] = g_class.getChangePermissions(geometry_id, 0)
                    case "right_lc":
                        data_vals[data_key] = g_class.getChangePermissions(geometry_id, 1)
                    case "max_speed":
                        if self.units.name in ['IMPERIAL', 'UK']: data_vals[data_key] = g_class.getMaxSpeed(geometry_id) * 2.2369362920544
                        else: data_vals[data_key] = g_class.getMaxSpeed(geometry_id) * 3.6
                    case _:
                        raise ValueError("Sim.get_vehicle_vals: Unrecognised key ('{0}').".format(data_key))
        
        if set(data_vals.keys()) != set(data_vals):
            raise ValueError("Sim.get_edge_vals: Invalid data_keys given (must be from accepted list).")
        if return_val: return list(data_vals.values())[0]
        else: return data_vals

    def set_geometry_vals(self, geometry_id, **kwargs):
        """
        Calls the TraCI API to change a edge or lane's state.
        :param geometry_id: Edge or lane ID
        :param setMaxSpeed: Set new max speed value [int|float]
        :param setAllowed: Set allowed vehicle types [(str)], empty list allows all (lane only)
        :param setDisallowed: Set disallowed vehicle types [(str)] (lane only)
        :param setChangePermissions: Set lane change permission with type list and direction [(str), ['LEFT'|'RIGHT']] (lane only)
        """
        
        if geometry_id in self.all_edges:   g_class, g_name = traci.edge, "edge"
        elif geometry_id in self.all_lanes: g_class, g_name = traci.lane, "lane"
        else: raise KeyError("Sim.get_edge_vals: Unrecognised egde or lane ID given ('{0}').".format(geometry_id))
        
        for command, value in kwargs.items():
            match command:
                case "max_speed":
                    if isinstance(value, int) or isinstance(value, float): 
                        if self.units.name in ['IMPERIAL', 'UK']: g_class.setMaxSpeed(geometry_id, value / 2.2369362920544)
                        else: g_class.setMaxSpeed(geometry_id, value / 3.6)
                    else: raise TypeError("Sim.set_geometry_vals ({0}): Invalid speed value '{1}' (must be [int|float], not '{2}').".format(command, value, type(value).__name__))

                case "allowed":
                    if g_name != "lane": raise ValueError("Sim.set_geometry_vals ({0}): Command is only valid for lanes.".format(command))
                    if hasattr(value, "__iter__"):
                        curr_allowed = list(g_class.getAllowed(geometry_id))
                        allowed = tuple(set(curr_allowed + list(value)))
                        g_class.setAllowed(geometry_id, allowed)
                        
                        curr_disallowed = list(g_class.getDisallowed(geometry_id))
                        disallowed = tuple(set(curr_disallowed) - set(value))
                        g_class.setDisallowed(geometry_id, disallowed)
                    else: raise TypeError("Sim.set_geometry_vals ({0}): Invalid type list value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__))

                case "disallowed":
                    if g_name != "lane": raise ValueError("Sim.set_geometry_vals ({0}): Command is only valid for lanes.".format(command))
                    if hasattr(value, "__iter__"):
                        curr_disallowed = list(g_class.getDisallowed(geometry_id))
                        disallowed = tuple(set(curr_disallowed + list(value)))
                        g_class.setDisallowed(geometry_id, disallowed)
                        
                        curr_allowed = list(g_class.getAllowed(geometry_id))
                        allowed = tuple(set(curr_allowed) - set(value))
                        g_class.setAllowed(geometry_id, allowed)
                    else: raise TypeError("Sim.set_geometry_vals ({0}): Invalid type list value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__))

                case "left_lc":
                    if g_name != "lane": raise ValueError("Sim.set_geometry_vals ({0}): Command is only valid for lanes.".format(command))
                    if hasattr(value, "__iter__"):
                        g_class.setChangePermissions(geometry_id, value[0], 1)
                    else: raise TypeError("Sim.set_geometry_vals ({0}): Invalid type list value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__))
                
                case "right_lc":
                    if g_name != "lane": raise ValueError("Sim.set_geometry_vals ({0}): Command is only valid for lanes.".format(command))
                    if hasattr(value, "__iter__"):
                        g_class.setChangePermissions(geometry_id, value[0], -1)
                    else: raise TypeError("Sim.set_geometry_vals ({0}): Invalid type list value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__))

                case _:
                    raise ValueError("Sim.set_vehicle_vals: Unrecognised command ('{0}').".format(command))
                
    def get_last_step_geometry_vehicles(self, geometry_ids, v_types = None, flatten = False) -> dict|list:
        """
        Get the IDs of vehicles on a lane or egde, by geometry ID.
        :param geometry_ids: Lane or edge ID or list of lane or edge IDs
        :param v_types: Included vehicle types
        :param flatten: If true, all IDs are returned in a 1D array, else a dict with vehicles for each lane or edge
        :return dict|list: Dict or list containing all vehicle IDs
        """

        geometry_ids = [geometry_ids] if not isinstance(geometry_ids, list) else geometry_ids
        if len(geometry_ids) == 1: flatten = True
        v_types = [v_types] if v_types != None and not isinstance(v_types, list) else v_types

        vehicle_ids = [] if flatten else {}
        for geometry_id in geometry_ids:

            g_vehicle_ids = self.get_geometry_vals(geometry_id, "vehicle_ids")
            old = g_vehicle_ids
            if v_types != None:
                g_vehicle_ids = [vehicle_id for vehicle_id in g_vehicle_ids if self.get_vehicle_vals(vehicle_id, "type") in v_types]

            if flatten: vehicle_ids += g_vehicle_ids
            else: vehicle_ids[geometry_id] = g_vehicle_ids

        if flatten: vehicle_ids = list(set(vehicle_ids))

        return vehicle_ids
    
    def geometry_exists(self, geometry_id) -> str|None:
        """
        Get geometry type by ID, if exists.
        :param geometry_id: Lane or edge ID
        :return str|None: Geometry type, or None if it does not exist.
        """

        if geometry_id in self.all_edges: return "edge"
        elif geometry_id in self.all_lanes: return "lane"
        else: return None

    def detector_exists(self, detector_id) -> str|None:
        """
        Get detector type by ID, if exists.
        :param detector_id: Detector ID
        :return str|None: Detector type, or None if it does not exist.
        """

        if detector_id in self.available_detectors.keys():
            return self.available_detectors[detector_id]
        else: return None

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

        if junc_params != None:
            if not isinstance(junc_params, dict) or not isinstance(junc_params, str):
                raise TypeError("Junc.init: Invalid junc_params (must be [dict|filepath (str)], not '{0}').".format(type(junc_params).__name__))
            elif isinstance(junc_params, str) and junc_params.endswith(".json"):
                if os.path.exists(junc_params):
                    with open(junc_params, "r") as fp:
                        junc_params = json.load(fp)
                else: raise FileNotFoundError("Junc.init: Junction parameters file '{0}' not found.".format(junc_params))
            else: raise ValueError("Junc.init: Invalid junction parameters file '{0}' (must be '.json' file).")

            if "inflow_detectors" not in junc_params.keys() or "outflow_detectors" not in junc_params.keys():
                raise KeyError("Junc.init: 'inflow_detectors' and 'outflow_detectors' are required junction parameters.")

            for detector_id in junc_params["inflow_detectors"]:
                if detector_id not in self.sim.available_detectors.keys():
                    raise KeyError("Junc.init: Unrecognised detector ID given in inflow_detectors ('{0}').".format(detector_id))
            for detector_id in junc_params["outflow_detectors"]:
                if detector_id not in self.sim.available_detectors.keys():
                    raise KeyError("Junc.init: Unrecognised detector ID given in outflow_detectors ('{0}').".format(detector_id))

            self.inflow_detectors = junc_params["inflow_detectors"]
            self.outflow_detectors = junc_params["outflow_detectors"]

            if "flow_vtypes" in junc_params.keys(): self.flow_vtypes = ["all"] + junc_params["flow_vtypes"]
            else: self.flow_vtypes = ["all"]

            self.avg_horizon = int(60 / self.sim_step_length) if "avg_horizon" not in junc_params.keys() else int(junc_params["avg_horizon"])
            
            self.v_in, self.v_out = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}
            self.avg_inflow, self.avg_outflow = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}
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
            self.avg_inflow, self.avg_outflow = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}
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
            step_v_in = self.sim.get_last_step_detector_vehicles(self.inflow_detectors, flatten=True)
            step_v_out = self.sim.get_last_step_detector_vehicles(self.outflow_detectors, flatten=True)

            for vtype in self.flow_vtypes:
                new_v_in = [v_id for v_id in step_v_in if v_id not in self.v_in[vtype] and (vtype == "all" or v_id.startswith(vtype))]
                self.v_in[vtype] += new_v_in
                self.inflows[vtype].append(len(new_v_in))
                self.avg_inflow[vtype].append((sum(self.inflows[vtype][-self.avg_horizon:]) / len(self.inflows[vtype][-self.avg_horizon:])) / self.sim_step_length)

            for vtype in self.flow_vtypes:
                new_v_out = [v_id for v_id in step_v_out if v_id not in self.v_out[vtype] and (vtype == "all" or v_id.startswith(vtype))]
                self.v_out[vtype] += new_v_out
                self.outflows[vtype].append(len(new_v_out))
                self.avg_outflow[vtype].append((sum(self.outflows[vtype][-self.avg_horizon:]) / len(self.outflows[vtype][-self.avg_horizon:])) / self.sim_step_length)

class Units(Enum):
    METRIC = 1
    IMPERIAL = 2
    UK = 3

def get_cumulative_arr(arr, start) -> list:
    for i in range(start, len(arr)):
        arr[i] += arr[i - 1]
    return arr