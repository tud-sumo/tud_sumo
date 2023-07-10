import os, sys, traci, json, math
from functools import reduce
from tqdm import tqdm
from random import randint
from copy import copy
from warnings import warn
import matplotlib.pyplot as plt

class Simulation:
    def __init__(self, main_junction = None, verbose = False) -> None:
        path_tools = os.path.join(os.environ.get("SUMO_HOME"), 'tools')

        if path_tools in sys.path: pass
        else: sys.path.append(path_tools)

        self.running = False
        self.gui = False
        self.verbose = verbose or '-v' in sys.argv

        self.junc_phases = None
        self.def_junc = main_junction

        self.available_detectors = {}
        self.step_length = 0.5

        self.get_individual_vehicle_data = True
        self.all_curr_vehicle_ids = []
        self.known_vehicles = {}
        self.light_changes = 0

    def start(self, config_file = None, net_file = None, route_file = None, add_file = None, cmd_options = None, step_len = None, warnings = False, gui = False) -> None:
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

        if config_file == net_file == None: raise ValueError("Either config or network file required.")
        
        if config_file != None:
            if config_file.endswith(".sumocfg"): sumoCMD += ["-c", config_file]
            else: raise ValueError("Invalid config file extension.")
        else:
            sumoCMD += ["-n", net_file]
            if route_file != None: sumoCMD += ["-r", route_file]
            if add_file != None: sumoCMD += ["-a", add_file]
            if cmd_options != None: sumoCMD += cmd_options
        
        if step_len != None: self.step_length = step_len
        sumoCMD += ["--step-length", str(self.step_length)]
        if not warnings or not self.verbose: sumoCMD.append("--no-warnings")

        traci.start(sumoCMD)
        self.running = True

        if self.junc_phases != None: self.update_lights()
        self.curr_step = 0
        self.time_val = traci.simulation.getTime()

        self.available_detectors = {detector_id: 'multientryexit' for detector_id in list(traci.multientryexit.getIDList())}
        self.available_detectors.update({detector_id: 'inductionloop' for detector_id in list(traci.inductionloop.getIDList())})
    
    def is_running(self, close=True):
        if traci.simulation.getMinExpectedNumber() == 0:
            if close: self.end()
            print('Ended simulation early: no vehicles remaining.')
            return False
        
        return True

    def end(self):
        traci.close()
        self.running = False

    def step_through(self, n_steps = 1, end_step = None, sim_dur = None, n_light_changes = None, detector_list = None, prev_data = None, vTypes = None, cumulative_data = False) -> dict:
        """
        Step through simulation from the current time until end_step, aggregating
        data during this period.
        :param n_steps:         Perform n steps of the simulation (defaults to 1)
        :param end_step:        End point for stepping through simulation (given instead of end_step)
        :param sim_dur:         Simulation duration
        :param n_light_changes: Run for n light changes (across all junctions)
        :param detector_list:   List of detector IDs to collect data from (defaults to all)
        :param prev_data:       If given, new data is appended to this dictionary
        :param vTypes:          Vehicle type(s) to collect data of (list of types or string, defaults to all)
        :param cumulative_data: Denotes whether to get cumulative veh count and TTS values
        :return dict:           All data collected through the time period, separated by detector
        """

        if detector_list == None: detector_list = list(self.available_detectors.keys())
        start_time = self.curr_step

        if end_step == None and n_steps != None: end_step = self.curr_step + n_steps
        elif end_step == None and sim_dur != None: end_step = self.curr_step + (sim_dur / self.step_length)
        elif end_step == None and n_light_changes != None: end_step, init_changes = math.inf, self.light_changes
        elif end_step == None: raise ValueError("No time value given for step_through() function.")

        if prev_data == None:
            prev_steps, all_data = 0, {"data": {"detector": {}, "vehicle": {}, "all_vehicles": []}, "step_len": self.step_length, "start": start_time}
        else: 
            prev_steps = set([len(data_arr) for data_arr in prev_data["data"]["vehicle"].values()] +
                            [len(detector_data["speeds"]) for detector_data in prev_data["data"]["detector"].values()] +
                            [len(detector_data["veh_counts"]) for detector_data in prev_data["data"]["detector"].values()])
            
            if len(prev_steps) != 1: raise ValueError("Invalid prev_data: different length arrays")
            else:
                prev_steps = prev_steps.pop()
                all_data = prev_data

        if self.verbose: pbar = tqdm(desc="Running simulation", total=end_step - self.curr_step)
        while self.curr_step < end_step:

            last_step_data, all_v_data = self.step(detector_list, vTypes)
            all_data["data"]["all_vehicles"].append(all_v_data)

            if len(all_data["data"]["detector"]) == 0:
                for detector_id in detector_list:
                    all_data["data"]["detector"][detector_id] = {"type": self.available_detectors[detector_id], "speeds": [], "veh_counts": [], "occupancies": []}
                all_data["data"]["vehicle"] = {"no_vehicles": [], "tts": [], "delay": []}

            for detector_id in last_step_data["detector"].keys():
                if detector_id not in all_data["data"]["detector"].keys(): raise KeyError("Unrecognised detector ID found: "+detector_id)
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

        return all_data

    def step(self, detector_list = None, vTypes = None) -> dict:
        """
        Increment simulation by one time step, updating light state.
        :param detector_list: List of detector IDs to collect data from
        :param vTypes:        Vehicle type(s) to collect data of (list of types or string, defaults to all)
        :return dict:         Simulation data
        """

        data = {"detector": {}, "vehicle": {}}
        
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
                elif phases["curr_time"] >= sum(phases["times"][:phases["curr_phase"] + 1]): # Can't handle a phase time < the time step
                    phases["curr_phase"] += 1                                                # Need to support this?
                    update_junc_lights.append(junction_id)

            self.update_lights(update_junc_lights)

        if detector_list == None: detector_list = list(self.available_detectors.keys())
        for detector_id in detector_list:
            data["detector"][detector_id] = {}
            if detector_id not in self.available_detectors.keys(): raise KeyError("KeyError: Unknown detector_id '"+detector_id+"'")
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

        return data, all_v_data
    
    def set_phases(self, junc_phases, start_phase = 0) -> None:
        """
        Sets the phases for the simulation.
        :param junc_phases: Can either be for a single junction (no junction ID given),
                            or for multiple junctions, with keys as junction ID.
        :param start_phase: Phase number to start at, defaults to 0
        """

        # junc_phases = {junction_0: {phases: [phase_0, phase_1], times: [time_0, time_1]}, ...}
        # where phases are light strings and times are phase length
        self.junc_phases = junc_phases

        if "phases" in self.junc_phases.keys():
            if self.def_junc != None: self.junc_phases = {self.def_junc: self.junc_phases}
            else:
                raise ValueError("Single phase settings given, but no junction ID.")

        for junc_phase in self.junc_phases.values():
            if "curr_phase" not in junc_phase.keys(): junc_phase["curr_phase"] = start_phase
            if junc_phase["curr_phase"] > len(junc_phase["phases"]): junc_phase["curr_phase"] -= len(junc_phase["phases"])

            junc_phase["curr_time"] = sum(junc_phase["times"][:junc_phase["curr_phase"]])
            junc_phase["cycle_len"] = sum(junc_phase["times"])

    def set_phase(self, junction, phase_no) -> None:
        """
        Change to a different phase at the specified junction
        :param junction_id: Junction ID
        :param phase_no: Phase number
        """
        
        if 0 < phase_no < len(self.junc_phases[junction]["phases"]):
            self.junc_phases[junction]["curr_phase"] = phase_no
            self.junc_phases[junction]["curr_time"] = sum(self.junc_phase["times"][:phase_no])

            self.update_lights(junction)

        else: raise ValueError("Invalid phase number '{0}', must be: [0-{1}]".format(phase_no, len(self.junc_phases[junction]["phases"])))

    def set_lights(self, junction_id, light_str) -> None:
        """
        Set lights to a specific setting. Will be overwritten at next
        light update.
        :param junction_id: Junction ID
        :param light_str: SUMO light string
        """
        traci.trafficlight.setRedYellowGreenState(junction_id, light_str)

    def update_lights(self, junction_ids = None) -> None:
        """
        Update light settings for given junctions
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
        Tests if vehicle exists in the network
        :return bool: True if ID in list of current vehicle IDs 
        """
        return vehicle_id in self.all_curr_vehicle_ids

    def get_last_step_vehicles(self, detector_ids = None) -> dict:
        """
        Get the IDs of vehicles that passed over the specified detectors
        :param detector_ids: detector ID or list of detector IDs (defaults to all)
        :return dict: Dict containing vehicle IDs for each detector
        """

        detector_ids = list(self.available_detectors.keys()) if detector_ids == None else detector_ids
        detector_ids = [detector_ids] if not isinstance(detector_ids, list) else detector_ids

        vehicle_ids = {}
        for detector_id in detector_ids:
            
            if detector_id not in self.available_detectors.keys(): raise KeyError("KeyError: Unknown detector_id '"+detector_id+"'")
            detector_type = self.available_detectors[detector_id]

            if detector_type == "inductionloop":
                vehicle_ids[detector_id] = list(traci.inductionloop.getLastStepVehicleIDs(detector_id))
            elif detector_type == "multientryexit":
                vehicle_ids[detector_id] = list(traci.multientryexit.getLastStepVehicleIDs(detector_id))
            else:
                warn("Warning: Unknown detector type '"+detector_type+"'")

        return vehicle_ids
    
    def get_vehicle_data(self, vehicle_id, vehicle_type = None):
        """
        Get data for specified vehicle, updating known_vehicles dict
        :param vehicle_id: vehicle ID
        :param vehicle_type: Vehicle type if known
        :return dict: Vehicle data dictionary, returns None if does not exist in simulation
        """

        if vehicle_id not in self.all_curr_vehicle_ids: return None

        if vehicle_type == None:
            if "cbikes" in vehicle_id: vehicle_type = "cbikes"
            elif "bikes" in vehicle_id: vehicle_type = "bikes"
            else: vehicle_type = "cars"

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

    def get_all_vehicle_data(self, types = None, type_from_id = True) -> dict:
        """
        Collects vehicle data from SUMO, by id and/or type (defaults to all vehicles)
        :param types: Type(s) of vehicles to fetch
        :return dict: Vehicle data by id
        """

        all_vehicle_data = {}
        total_vehicle_data = {"no_vehicles": 0, "no_waiting": 0}

        for vehicle_id in self.all_curr_vehicle_ids:

            # Saving known vehicles reduces calls to traci by not
            # fetching already known (& unchanging!) data
            if vehicle_id in self.known_vehicles.keys(): vehicle_type = self.known_vehicles[vehicle_id]["type"]
            else:
                vehicle_type = traci.vehicle.getVehicleClass(vehicle_id)

            if types is None or (isinstance(types, list) and vehicle_type in types) or (isinstance(types, str) and vehicle_type == types):

                if self.get_individual_vehicle_data: vehicle_data = self.get_vehicle_data(vehicle_id, vehicle_type)
                
                all_vehicle_data[vehicle_id] = vehicle_data
                    
                total_vehicle_data["no_vehicles"] += 1
                if vehicle_data["speed"] < 0.1: total_vehicle_data["no_waiting"] += 1

        return total_vehicle_data, all_vehicle_data
    
def get_cumulative_arr(arr, start) -> list:
    for i in range(start, len(arr)):
        arr[i] += arr[i - 1]
    return arr