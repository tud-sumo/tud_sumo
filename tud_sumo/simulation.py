import os, sys, traci, sumolib, json, math
from tqdm import tqdm
from copy import copy, deepcopy
from events import EventScheduler, Event
from controllers import VSLController, RGController
from shapely.geometry import LineString, Point
from utils import *

class Simulation:
    def __init__(self, scenario_name: str|None = None) -> None:
        """
        :param scenario_name: Scenario label saved to simulation object (defaults to name of '.sumocfg')
        """

        path_tools = os.path.join(os.environ.get("SUMO_HOME"), 'tools')

        if path_tools in sys.path: pass
        else: sys.path.append(path_tools)

        self.scenario_name = scenario_name
        self.running = False
        self.curr_step = 0
        self.gui = False

        self.junc_phases = None

        self.track_juncs = False
        self.all_juncs = []
        self.tracked_juncs = {}

        self.all_edges = None
        self.all_lanes = None
        self.all_routes = None

        self.tracked_edge_ids = []
        self.tracked_edges = {}

        self.available_detectors = {}
        self.step_length = None

        self.get_individual_vehicle_data = True
        self.all_curr_vehicle_ids = set([])
        self.all_loaded_vehicle_ids = set([])
        self.known_vehicles = {}
        self.light_changes = 0

        self.all_data = None

        self.scheduler = None

        self.controllers = {}

    def start(self, config_file: str|None = None, net_file: str|None = None, route_file: str|None = None, add_file: str|None = None, cmd_options: list|None = None, units: int = 1, get_individual_vehicle_data: bool = True, suppress_warnings: bool = False, ignore_TraCI_err: bool = False, gui: bool = False) -> None:
        """
        Intialises SUMO simulation.
        :param config_file: Location of '.sumocfg' file (can be given instead of net_file)
        :param net_file:    Location of '.net.xml' file (can be given instead of config_file)
        :param route_file:  Location of '.rou.xml' route file
        :param add_file:    Location of '.add.xml' additional file
        :param cmd_options: List of any other command line options
        :param units:       Data collection units [1 (metric) | 2 (IMPERIAL) | 3 (UK)] (defaults to 'metric')
        :param get_individual_vehicle_data: Denotes whether to get individual vehicle data (set to False to improve performance)
        :param suppress_warnings: Suppress simulation warnings
        :param ignore_TraCI_err: If true and a fatal traCI error occurs, the simulation ends but the program continues to run
        :param gui:         Bool denoting whether to run GUI
        """

        self.gui = gui
        sumoCMD = ["sumo-gui"] if gui else ["sumo"]

        if config_file == net_file == None: raise ValueError("(step {0}) Simulation.start(): Either config or network file required.".format(self.curr_step))
        
        if config_file != None:
            if config_file.endswith(".sumocfg"):
                sumoCMD += ["-c", config_file]
                self.scenario_name = get_scenario_name(config_file)
            else: raise ValueError("(step {0}) Simulation.start(): Invalid config file extension.".format(self.curr_step))
        else:
            sumoCMD += ["-n", net_file]
            self.scenario_name = get_scenario_name(net_file)
            if route_file != None: sumoCMD += ["-r", route_file]
            if add_file != None: sumoCMD += ["-a", add_file]
        
        if cmd_options != None: sumoCMD += cmd_options

        if units in [1, 2, 3]: self.units = Units(units)
        else: raise ValueError("(step {0}) Plot.init: Invalid simulation units '{1}' (must be 1-3)".format(self.curr_step, units))

        traci.start(sumoCMD)
        self.running = True

        self.step_length = float(traci.simulation.getOption("step-length"))

        if self.junc_phases != None: self.update_lights()
        self.time_val = traci.simulation.getTime()

        for detector_id in list(traci.multientryexit.getIDList()):
            self.available_detectors[detector_id] = {'type': 'multientryexit', 'position': {'entry_lanes': traci.multientryexit.getEntryLanes(detector_id),
                                                                                            'exit_lanes': traci.multientryexit.getExitLanes(detector_id),
                                                                                            'entry_positions': traci.multientryexit.getEntryPositions(detector_id),
                                                                                            'exit_positions': traci.multientryexit.getExitPositions(detector_id)}}
            
        for detector_id in list(traci.inductionloop.getIDList()):
            self.available_detectors[detector_id] = {'type': 'inductionloop', 'position': {'lane_id': traci.inductionloop.getLaneID(detector_id), 'position': traci.inductionloop.getPosition(detector_id)}}

        self.all_edges = traci.edge.getIDList()
        self.all_lanes = traci.lane.getIDList()
        self.all_routes = traci.route.getIDList()

        self.get_individual_vehicle_data = get_individual_vehicle_data

        self.ignore_TraCI_err = ignore_TraCI_err
        self.suppress_warnings = suppress_warnings
        
    def start_junc_tracking(self, juncs: str|list|dict|None = None) -> None:
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
            elif isinstance(juncs, (list, tuple)):
                junc_ids, junc_params = juncs, None
            elif isinstance(juncs, str):
                junc_ids, junc_params = [juncs], None
            else: raise TypeError("(step {0}) Simulation.start_junc_tracking(): Invalid junc_params (must be [str|list|dict], not '{1}').".format(self.curr_step, type(juncs).__name__))

            if len(set(all_juncs).intersection(set(junc_ids))) != len(junc_ids):
                raise KeyError("(step {0}) Simulation.start_junc_tracking(): Junction ID not found.".format(self.curr_step))
            else: track_list = junc_ids

        self.all_juncs, self.all_tls = all_juncs, all_tls

        for junc_id in track_list:
            junc_param = junc_params[junc_id] if junc_params != None else None
            self.tracked_juncs[junc_id] = TrackedJunction(junc_id, self, junc_param)
            self.tracked_juncs[junc_id].update_vals = True

    def reset_data(self) -> None:
        """
        Resets data collection.
        """

        for junction in self.tracked_juncs.values():
            junction.reset()

        for edge in self.tracked_edges.values():
            edge.reset()

        for controller in self.controllers.values():
            controller.reset()

        self.all_data = None

    def is_running(self, close: bool=True) -> bool:
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

    def end(self) -> None:
        """
        Ends the simulation.
        """

        traci.close()
        self.running = False

    def save_data(self, filename: str|None = None, overwrite: bool = True) -> None:
        """
        Save all vehicle, detector and junction data in a JSON file.
        :param filename:  Output filepath (defaults to ./'scenario_name'.json)
        :param overwrite: Prevent previous outputs being overwritten
        """

        if filename == None: filename = self.scenario_name
        if not filename.endswith(".json"): filename += ".json"

        if os.path.exists(filename) and overwrite:
            if not self.suppress_warnings: print("(step {0}) (WARNING) Simulation.save_data(): Simulation.save_data: File '{1}' already exists and will be overwritten".format(self.curr_step, filename))
        elif os.path.exists(filename) and not overwrite:
            raise FileExistsError("(step {0}) Simulation.save_data(): File '{1}' already exists and can't be overwritten.".format(self.curr_step, filename))

        if self.all_data != None:
            if self.scheduler != None: self.all_data["data"]["events"] = self.scheduler.__dict__()
            with open(filename, "w") as fp:
                json.dump(self.all_data, fp, indent=4)
        else: raise AssertionError("(step {0}) Simulation.save_data(): No data to save as simulation has not been run.".format(self.curr_step))

    def start_edge_tracking(self, edge_list):
        self.tracked_edge_ids = edge_list
        for edge_id in edge_list:
            if self.geometry_exists(edge_id) == None:
                raise KeyError("(step {0}) Simulation.start_edge_tracking(): Geometry ID '{1}' not found.".format(self.curr_step, edge_id))
            self.tracked_edges[edge_id] = TrackedEdge(edge_id, self)

    def add_events(self, event_params: Event|list|dict|str) -> None:
        """
        Add events and event scheduler.
        :param event_parms: Event parameters [Event|[Event]|dict|filepath]
        """
        if self.scheduler == None:
            self.scheduler = EventScheduler(self)
        self.scheduler.add_events(event_params)

    def add_controllers(self, controller_params: str|dict) -> dict:
        """
        Add controllers from parameters in a dictionary/JSON file.
        :param controller_params: Controller parameters dictionary or filepath
        """

        controller_params = load_params(controller_params, "Simulation.add_controllers()", self.curr_step, "controller_params")

        for c_id, c_params  in controller_params.items():
            if isinstance(c_params, (RGController, VSLController)):
                self.controllers[c_id] = c_params
            elif isinstance(c_params, dict):
                if 'type' not in c_params.keys(): raise KeyError("(step {0}) Simulation.add_controllers(): No type given (must be [1 (RG)|2 (VSL)]).".format(self.curr_step))
                if c_params['type'] in [1, 2]:

                    c_type = Controller(c_params['type'])
                    match c_type.name:
                        case 'RG': controller = RGController(c_id, c_params, self)
                        case 'VSL': controller = VSLController(c_id, c_params, self)
                    
                    self.controllers[c_id] = controller

                else: ValueError("(step {0}) Simulation.add_controllers(): Invalid controller type (must be [1 (RG)|2 (VSL)]).".format(self.curr_step))
            else: raise TypeError("(step {0}) Simulation.add_controllers(): Invalid parameters type in dictionary (must be [dict|RGController|VSLController], not '{1}').".format(self.curr_step, type(c_params).__name__))

        return self.controllers

    def step_through(self, n_steps: int = 1, end_step: int|None = None, sim_dur: int|None = None, n_light_changes: int|None = None, detector_list: list|None = None, vTypes: list|None = None, keep_data: bool = True, append_data: bool = True, cumulative_data: bool = False) -> dict:
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

        if not self.is_running(): return

        if append_data == True: prev_data = self.all_data
        else: prev_data = None

        if detector_list == None: detector_list = list(self.available_detectors.keys())
        start_time = self.curr_step

        if end_step == None and n_steps != None: end_step = self.curr_step + n_steps
        elif end_step == None and sim_dur != None: end_step = self.curr_step + (sim_dur / self.step_length)
        elif end_step == None and n_light_changes != None: end_step, init_changes = math.inf, self.light_changes
        elif end_step == None: raise ValueError("(step {0}) Simulation.step_through(): No time value given.".format(self.curr_step))

        if prev_data == None:
            prev_steps, all_data = 0, {"data": {}, "scenario_name": self.scenario_name, "step_len": self.step_length, "units": self.units.name, "start": start_time}
            
            if len(self.available_detectors) > 0: all_data["data"]["detector"] = {}
            if self.track_juncs: all_data["data"]["junctions"] = {}
            if len(self.tracked_edges) > 0: all_data["data"]["edges"] = {}
            if len(self.controllers) > 0: all_data["data"]["controllers"] = {}
            all_data["data"]["vehicle"] = {}
            if self.get_individual_vehicle_data: all_data["data"]["all_vehicles"] = []
            if self.scheduler != None: all_data["data"]["events"] = {}
        else: 
            prev_steps = set([len(data_arr) for data_arr in prev_data["data"]["vehicle"].values()] +
                            [len(detector_data["speeds"]) for detector_data in prev_data["data"]["detector"].values()] +
                            [len(detector_data["veh_counts"]) for detector_data in prev_data["data"]["detector"].values()])
            
            if len(prev_steps) != 1: raise ValueError("(step {0}) Simulation.step_through(): Invalid prev_data (different length arrays).".format(self.curr_step))
            else:
                prev_steps = prev_steps.pop()
                all_data = prev_data

        if not self.gui and n_steps > 1: pbar = tqdm(desc="Running simulation", total=end_step - self.curr_step)
        while self.curr_step < end_step:

            try: last_step_data, all_v_data = self.step(detector_list, vTypes)
            except traci.exceptions.FatalTraCIError:
                self.running = False
                if self.ignore_TraCI_err:
                    if not self.suppress_warnings: print("(step {0}) (WARNING) Simulation.step_through(): Fatal TraCI connection error occured and 'Sim.ignore_TraCI_err' is set to True.".format(self.curr_step))
                    break
                else: raise traci.exceptions.FatalTraCIError("(step {0}) Simulation.step_through(): Fatal TraCI connection error occured.".format(self.curr_step))

            if self.get_individual_vehicle_data: all_data["data"]["all_vehicles"].append(all_v_data)
            for controller in self.controllers.values(): controller.update()
            for edge in self.tracked_edges.values(): edge.update()

            if len(all_data["data"]["detector"]) == 0:
                for detector_id in detector_list:
                    all_data["data"]["detector"][detector_id] = self.available_detectors[detector_id]
                    all_data["data"]["detector"][detector_id].update({"speeds": [], "veh_counts": [], "veh_ids": [], "occupancies": []})
                all_data["data"]["vehicle"] = {"no_vehicles": [], "tts": [], "delay": []}

            for detector_id in last_step_data["detector"].keys():
                if detector_id not in all_data["data"]["detector"].keys(): raise KeyError("(step {0}) Simulation.step_through(): Unrecognised detector ID found ('{1}').".format(self.curr_step, detector_id))
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
        if self.track_juncs: all_data["data"]["junctions"] = last_step_data["junctions"]
        if self.scheduler != None: all_data["data"]["events"] = self.scheduler.__dict__()
        for e_id, edge in self.tracked_edges.items(): all_data["data"]["edges"][e_id] = edge.get_curr_data()
        for c_id, controller in self.controllers.items(): all_data["data"]["controllers"][c_id] = controller.get_curr_data()

        if keep_data:
            self.all_data = all_data
            return all_data
        else:
            self.reset_data()
            return None

    def step(self, detector_list: list|None = None, vTypes: list|None = None) -> dict:
        """
        Increment simulation by one time step, updating light state. step_through is recommended to run the simulation.
        :param detector_list: List of detector IDs to collect data from
        :param vTypes:        Vehicle type(s) to collect data of (list of types or string, defaults to all)
        :return dict:         Simulation data
        """

        data = {"detector": {}, "vehicle": {}}
        if self.track_juncs: data["junctions"] = {}
        
        traci.simulationStep()
        time_diff = traci.simulation.getTime() - self.time_val
        self.time_val = traci.simulation.getTime()

        self.all_curr_vehicle_ids = set(traci.vehicle.getIDList())
        self.all_loaded_vehicle_ids = set(traci.vehicle.getLoadedIDList())
        self.all_to_depart_vehicle_ids = self.all_loaded_vehicle_ids - self.all_curr_vehicle_ids

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
            if detector_id not in self.available_detectors.keys(): raise KeyError("(step {0}) Simulation.step(): Unrecognised detector ID found ('{1}').".format(self.curr_step, detector_id))
            if self.available_detectors[detector_id]["type"] == "multientryexit":

                detector_data = self.get_last_step_detector_data(detector_id, ["speed", "veh_count"])
                data["detector"][detector_id]["speeds"] = detector_data["speed"]
                data["detector"][detector_id]["veh_counts"] = detector_data["veh_count"]
                
            elif self.available_detectors[detector_id]["type"] == "inductionloop":

                detector_data = self.get_last_step_detector_data(detector_id, ["speed", "veh_count", "occupancy"])
                data["detector"][detector_id]["speeds"] = detector_data["speed"]
                data["detector"][detector_id]["veh_counts"] = detector_data["veh_count"]
                data["detector"][detector_id]["occupancies"] = detector_data["occupancy"]

            else:
                if not self.suppress_warnings: print("(step {0}) (WARNING) Simulation.step(): Unknown detector type '{1}'.".format(self.curr_step, self.available_detectors[detector_id]["type"]))

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

    def get_last_step_detector_vehicles(self, detector_ids: list|str, v_types: list|None = None, flatten: bool = False) -> dict|list:
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
            
            if detector_id not in self.available_detectors.keys(): raise KeyError("(step {0}) Simulation.get_last_step_detector_vehicles(): Detector ID '{1}' not found.".format(self.curr_step, detector_id))
            detector_type = self.available_detectors[detector_id]["type"]

            if detector_type == "inductionloop":
                detected_vehicles = list(traci.inductionloop.getLastStepVehicleIDs(detector_id))
            elif detector_type == "multientryexit":
                detected_vehicles = list(traci.multientryexit.getLastStepVehicleIDs(detector_id))
            else:
                raise KeyError("(step {0}) Simulation.get_last_step_detector_vehicles(): Unknown detector type '{1}'".format(self.curr_step, detector_type))
            
            if v_types != None:
                detected_vehicles = [vehicle_id for vehicle_id in detected_vehicles if self.get_vehicle_vals(vehicle_id, "type") in v_types]

            if flatten: vehicle_ids += detected_vehicles
            else: vehicle_ids[detector_id] = detected_vehicles

        if flatten: vehicle_ids = list(set(vehicle_ids))

        return vehicle_ids
    
    def get_last_step_detector_data(self, detector_id: int|str, data_keys: str|list) -> int|float|dict:
        """
        Get data values from a specific detector using a list of data keys.
        :param detector_id: Detector ID
        :param data_keys: List of keys from [veh_count|speed|halting_no (MEE only)|occupancy (IL only)|last_detection (IL only)], or single key
        :return dict: Values by data_key (or single value)
        """

        if detector_id not in self.available_detectors.keys():
            raise KeyError("(step {0}) Simulation.get_last_step_detector_data(): Detector with ID '{1}' not found.".format(self.curr_step, detector_id))
        else:
            detector_type = self.available_detectors[detector_id]["type"]

            match detector_type:
                case "multientryexit": d_class = traci.multientryexit
                case "inductionloop": d_class = traci.inductionloop
                case "_": raise ValueError("(step {0}) Simulation.get_last_step_detector_data(): Only 'multientryexit' and 'inductionloop' detectors are currently supported (not '{1}').".format(self.curr_step, detector_type))
            
        detector_data = {}
        if not isinstance(data_keys, (list, tuple)): data_keys = [data_keys]
        for data_key in data_keys:

            match data_key:
                case "veh_count":
                    detector_data[data_key] = d_class.getLastStepVehicleNumber(detector_id)
                case "speed":
                    speed = d_class.getLastStepMeanSpeed(detector_id)
                    if speed > 0:
                        if self.units.name in ['IMPERIAL', 'UK']: speed *= 2.2369362920544
                        else: speed *= 3.6
                    detector_data[data_key] = speed
                case _:
                    known_key = True
                    if detector_type == "multientryexit":
                        if data_key == "halting_no":
                            detector_data[data_key] = d_class.getLastStepHaltingNumber(detector_id)
                        else: known_key = False
                    else:
                        if data_key == "occupancy":
                            detector_data[data_key] = d_class.getLastStepOccupancy(detector_id)
                        elif data_key == "last_detection":
                            detector_data[data_key] = d_class.getTimeSinceDetection(detector_id)
                        else: known_key = False
                    if not known_key:
                        raise KeyError("(step {0}) Simulation.get_last_step_detector_data(): Unrecognised data key ('{1}').".format(self.curr_step, data_key))
                    
        if len(data_keys) == 1: return detector_data[data_keys[0]]
        else: return detector_data

    def get_interval_detector_data(self, detector_id: int|str, n_steps: int, data_keys: str|list, avg_vals: bool = False) -> float|dict:
        """
        Get data previously collected by a detector over range (curr step - n_step -> curr_step).
        :param detector_id: Detector ID
        :param n_steps: Interval length in steps (max at number of steps the simulation has run)
        :param data_keys: List of keys from [veh_counts|speeds|occupancy (IL only)], or single key
        :param avg_bool: Bool denoting whether to average values
        :return float|dict: Either single value or dict containing values by data_key
        """

        if self.all_data == None: raise AssertionError("(step {0}) Simulation.get_interval_detector_data(): No detector data as the simulation has not been run or data has been reset.".format(self.curr_step))
        elif detector_id not in self.available_detectors.keys() or detector_id not in self.all_data["data"]["detector"].keys():
            raise KeyError("(step {0}) Simulation.get_interval_detector_data(): Detector with ID '{1}' not found.".format(self.curr_step, detector_id))

        detector_data = {}
        if not isinstance(data_keys, (list, tuple)): data_keys = [data_keys]
        for data_key in data_keys:
            if data_key in ["veh_counts", "speeds", "occupancies"]:
                if data_key in self.all_data["data"]["detector"][detector_id].keys():
                    data_arr = self.all_data["data"]["detector"][detector_id][data_key][-n_steps:]
                    if avg_vals: data_arr = sum(data_arr) / len(data_arr)
                else: raise KeyError("(step {0}) Simulation.get_interval_detector_data(): Detector '{1}' of type '{2}' does not collect '{3}' data.".format(self.curr_step, detector_id, self.all_data["data"]["detector"][detector_id]["type"], data_key))
            else: raise KeyError("(step {0}) Simulation.get_interval_detector_data(): Unrecognised data key ('{1}').".format(self.curr_step, data_key))
            detector_data[data_key] = data_arr

        if len(data_keys) == 1: return detector_data[data_keys[0]]
        else: return detector_data

    def set_phases(self, new_junc_phases: dict, start_phase: int = 0, overwrite: bool = True) -> None:
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
                raise KeyError("(step {0}) Simulation.set_phases(): Junction with ID '{1}' does not exist, or it does not have a traffic light.".format(self.curr_step, junc_id))

            junc_phase = self.junc_phases[junc_id]

            for t in junc_phase["times"]:
                if t <= self.step_length: raise ValueError("(step {0}) Simulation.set_phases(): Invalid phase duration (phase_dur ({1}) <= resolution ({2})).".format(self.curr_step, t, self.step_length))

            if "curr_phase" not in junc_phase.keys(): junc_phase["curr_phase"] = start_phase
            if junc_phase["curr_phase"] > len(junc_phase["phases"]): junc_phase["curr_phase"] -= len(junc_phase["phases"])

            junc_phase["curr_time"] = sum(junc_phase["times"][:junc_phase["curr_phase"]])
            junc_phase["cycle_len"] = sum(junc_phase["times"])

        self.update_lights(new_junc_phases.keys())

    def set_tl_colour(self, junction_id: str|int, colour_str: str) -> None:
        """
        Sets a junction to a set colour/phase string for an indefinite amount of time. Can be used when tracking phases separately.
        :param junction_id: Junction ID
        :param colour_str: Colour character ([r|y|g|-]) or phase string ([r|y|g|-]+)
        """
        
        if junction_id not in list(traci.trafficlight.getIDList()):
            raise KeyError("(step {0}) Simulation.set_tl_colour(): Junction with ID '{1}' does not exist, or it does not have a traffic light.".format(self.curr_step, junction_id))
        else:
            if junction_id in self.tracked_juncs.keys():
                m_len = self.tracked_juncs[junction_id].m_len
            else:
                state_str = traci.trafficlight.getRedYellowGreenState(junction_id)
                m_len = len(state_str)

        if not isinstance(colour_str, str): raise TypeError("(step {0}) Simulation.set_tl_colour(): Invalid colour_str (must be 'str', not '{1}').".format(self.curr_step, type(colour_str).__name__))
        
        if len(colour_str) == 1:
            junc_phases = {junction_id: {"phases": [colour_str*m_len], "times": [math.inf]}}
        elif len(colour_str) == m_len:
            junc_phases = {junction_id: {"phases": [colour_str], "times": [math.inf]}}
        else:
            raise ValueError("(step {0}) Simulation.set_tl_colour(): Invalid colour_str (must be char or len(str) = junction movements length).".format(self.curr_step))
        
        self.set_phases(junc_phases, overwrite=False)

    def set_tl_metering_rate(self, junction_id: str|int, flow_rate: int|float, min_red: int|float = 1, y_dur: int|float = 1, green_time: int|float = 1) -> dict:
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
            raise KeyError("(step {0}) Simulation.set_tl_metering_rate(): Junction with ID '{1}' does not exist, or it does not have a traffic light.".format(self.curr_step, junction_id))
        else:
            if junction_id in self.tracked_juncs.keys():
                m_len = self.tracked_juncs[junction_id].m_len
            else:
                state_str = traci.trafficlight.getRedYellowGreenState(junction_id)
                m_len = len(state_str)

        if self.track_juncs and junction_id in self.tracked_juncs.keys():
            self.tracked_juncs[junction_id].metering_rates.append(flow_rate)
            self.tracked_juncs[junction_id].rate_times.append(self.curr_step)

        if min_red < self.step_length:
            if not self.suppress_warnings: print("(step {0}) (WARNING) Simulation.set_tl_metering_rate(): Minimum red duration ({1}s) is being set to step length ({2}s).".format(self.curr_step, min_red, self.step_length))
            min_red = self.step_length

        if flow_rate > 0:

            flow_pm = flow_rate / 60
            cycle_length = 60 / flow_pm

            red_time = cycle_length - green_time - y_dur
            if red_time < min_red or red_time <= 0:
                phases_dict = {"phases": ['G'*m_len], "times": [cycle_length]}

            else:
                phases_dict = {"phases": ['G'*m_len, 'y'*m_len, 'r'*m_len],
                            "times":  [green_time / self.step_length, y_dur / self.step_length, red_time / self.step_length]}
                
        elif flow_rate == 0:
            phases_dict = {"phases": ['R'*m_len], "times": [math.inf]}
        
        else: raise ValueError("(step {0}) Simulation.set_tl_metering_rate(): Flow rate ({1}) must be above 0.".format(self.curr_step, flow_rate))

        self.set_phases({junction_id: phases_dict}, overwrite=False)
        
        return phases_dict

    def change_phase(self, junction_id: str|int, phase_no: int) -> None:
        """
        Change to a different phase at the specified junction_id.
        :param junction_id: Junction ID
        :param phase_no: Phase number
        """
        
        if 0 < phase_no < len(self.junc_phases[junction_id]["phases"]):
            self.junc_phases[junction_id]["curr_phase"] = phase_no
            self.junc_phases[junction_id]["curr_time"] = sum(self.junc_phase["times"][:phase_no])

            self.update_lights(junction_id)

        else: raise ValueError("(step {0}) Simulation.change_phase(): Invalid phase number '{1}' (must be [0-{2}]).".format(self.curr_step, phase_no, len(self.junc_phases[junction_id]["phases"])))

    def update_lights(self, junction_ids: list|str = None) -> None:
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

    def vehicle_exists(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle exists in the network and has departed.
        :return bool: True if ID in list of current vehicle IDs 
        """
        return vehicle_id in self.all_curr_vehicle_ids
    
    def vehicle_loaded(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle is loaded (may not have departed).
        :return bool: True if ID in list of loaded vehicle IDs
        """
        return vehicle_id in self.all_loaded_vehicle_ids
    
    def vehicle_departed(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle is loaded but has not departed yet.
        :return bool: True if vehicle has not departed yet
        """
        if not self.vehicle_loaded(vehicle_id):
            raise KeyError("(step {0}) Simulation.vehicle_departed(): Vehicle with ID '{1}' has not been loaded.".format(self.curr_step, vehicle_id))
        return vehicle_id in self.all_to_depart_vehicle_ids
    
    def set_vehicle_vals(self, vehicle_id: str, **kwargs) -> None:
        """
        Calls the TraCI API to change a vehicle's state.
        :param vehicle_id: Vehicle ID
        :param highlight: Highlights vehicle with specified colour [str|(int)]
        :param speed: Set new speed value [int|float]
        :param max_speed: Set new max speed value [int|float]
        :param acceleration: Set acceleration for a given duration ([int|float], [int|float])
        :param lane_idx: Try and change lane for a given duration (int, [int|float])
        :param target: Set vehicle destination edge ID [str]
        :param route_id: Set vehicle route by route ID or list of edges (str)
        :param route_edges: Set vehicle route by list of edges ([str])
        """
        
        if not self.vehicle_exists(vehicle_id):
            raise KeyError("(step {0}) Simulation.set_vehicle_vals(): Unrecognised vehicle ID given ('{1}').".format(self.curr_step, vehicle_id))
        
        for command, value in kwargs.items():
            match command:
                case "highlight":
                    if value != None:
                        colour = value
                        if isinstance(colour, str):
                            if "#" in colour: colour = colour.lstrip("#")
                            if len(colour) != 6: raise ValueError("(step {0}) Simulation.set_vehicle_vals():({1}): '{2}' is not a valid hex colour.".format(self.curr_step, command, colour))
                            colour = tuple(int(colour[i:i+2], 16) for i in (0, 2, 4))
                        elif not isinstance(colour, (list, tuple)):
                            raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid colour (must be [str|list|tuple], not '{2}').".format(self.curr_step, command, type(colour).__name__))
                        elif len(colour) not in [3, 4] or all(x > 255 for x in colour) or all(x < 0 for x in colour):
                            raise ValueError("(step {0}) Simulation.set_vehicle_vals() ({1}): '{2}' is not a valid RGB or RGBA colour.".format(self.curr_step, command, colour))
                        
                        if len(colour) == 3: colour = list(colour) + [255]

                        traci.vehicle.setColor(vehicle_id, colour)
                    else:
                        v_type = self.get_vehicle_vals(vehicle_id, "type")
                        type_colour = tuple(traci.vehicletype.getColor(v_type))
                        traci.vehicle.setColor(vehicle_id, type_colour)

                case "speed":
                    if isinstance(value, (int, float)): 
                        if self.units.name in ['IMPERIAL', 'UK']: traci.vehicle.setSpeed(vehicle_id, value / 2.2369362920544)
                        else: traci.vehicle.setSpeed(vehicle_id, value / 3.6)
                    else: raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid speed value '{2}' (must be [int|float], not '{3}').".format(self.curr_step, command, value, type(value).__name__))
                
                case "max_speed":
                    if isinstance(value, (int, float)):
                        if self.units.name in ['IMPERIAL', 'UK']: traci.vehicle.setMaxSpeed(vehicle_id, value / 2.2369362920544)
                        else: traci.vehicle.setMaxSpeed(vehicle_id, value / 3.6)
                    else: raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid max speed value '{2}' (must be [int|float], not '{3}').".format(self.curr_step, command, value, type(value).__name__))
                
                case "acceleration":
                    if isinstance(value, (list, tuple)) and len(value) == 2:
                        if not isinstance(value[0], (int, float)):
                            raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid acceleration '{2}' (must be [int|float], not '{3}').".format(self.curr_step, command, value[0], type(value[0]).__name__))
                        if not isinstance(value[1], (int, float)):
                            raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid laneIndex '{2}' (must be [int|float], not '{3}').".format(self.curr_step, command, value[1], type(value[1]).__name__))
                        traci.vehicle.setAcceleration(vehicle_id, float(value[0]), float(value[1]))
                    else: raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): '{1}' requires 2 parameters (acceleration [int|float], duration [int|float])".format(self.curr_step, command))

                case "lane_idx":
                    if isinstance(value, (list, tuple)) and len(value) == 2:
                        if not isinstance(value[0], int): raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid laneIndex '{2}' (must be int, not '{3}').".format(self.curr_step, command, value[0], type(value[0]).__name__))
                        if not isinstance(value[1], (int, float)):
                            raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid duration '{2}' (must be [int|float], not '{3}').".format(self.curr_step, command, value[1], type(value[1]).__name__))
                        traci.vehicle.changeLane(vehicle_id, value[0], float(value[1]))
                    else: raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): '{1}' requires 2 parameters (laneIndex [int], duration [int|float])".format(self.curr_step, command))

                case "target":
                    if isinstance(value, str):
                        if value not in self.all_edges: raise KeyError("(step {0}) Simulation.set_vehicle_vals() ({1}): Edge ID '{2}' not found.".format(self.curr_step, command, value))
                        traci.vehicle.changeTarget(vehicle_id, value)
                    else: raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid edge ID '{2}' (must be str, not '{3}').".format(self.curr_step, command, value, type(value).__name__))
                
                case "route_id":
                    if isinstance(value, str):
                        if value not in self.all_routes: raise KeyError("(step {0}) Simulation.set_vehicle_vals() ({1}): Route ID '{2}' not found.".format(self.curr_step, command, value))
                        traci.vehicle.setRouteID(vehicle_id, value)
                    else: raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid route ID value '{2}' (must be str, not '{3}').".format(self.curr_step, command, value, type(value).__name__))
                
                case "route_edges":
                    if isinstance(value, (list, tuple)) and all(isinstance(x, str) for x in value):
                        for e_id in value:
                            if e_id not in self.all_edges: raise KeyError("(step {0}) Simulation.set_vehicle_vals() ({1}): Edge ID '{2}' in route edges not found.".format(self.curr_step, command, e_id))
                        traci.vehicle.setRoute(value)
                    else: raise TypeError("(step {0}) Simulation.set_vehicle_vals() ({1}): Invalid route egdes value '{2}' (must be (str), not '{3}').".format(self.curr_step, command, value, type(value).__name__))

                case _:
                    raise ValueError("(step {0}) Simulation.set_vehicle_vals(): Unrecognised command ('{1}').".format(self.curr_step, command))

    def get_vehicle_vals(self, vehicle_id: str, data_keys: str|list) -> dict|str|int|float|list:
        """
        Get data values for specific vehicle using a list of data keys.
        :param vehicle_id: Vehicle ID
        :param data_keys: List of keys from [type|length|speed|max_speed|acceleration|position|heading|ts|lane_idx|target|route_id|route_idx|route_edges], or single key
        :return dict: Values by data_key (or single value)
        """

        if not self.vehicle_exists(vehicle_id):
            raise KeyError("(step {0}) Simulation.get_vehicle_vals(): Unrecognised vehicle ID given ('{1}').".format(self.curr_step, vehicle_id))

        return_val = False
        if isinstance(data_keys, str):
            data_keys = [data_keys]
            return_val = True
        elif not isinstance(data_keys, (list, tuple)):
            raise TypeError("(step {0}) Simulation.get_vehicle_vals(): Invalid data_keys given '{1}' (must be [str|(str)], not '{2}').".format(self.curr_step, data_keys, type(data_keys).__name__))
        
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
                case "route_idx":
                    data_vals[data_key] = traci.vehicle.getRouteIndex(vehicle_id)
                case "route_edges":
                    data_vals[data_key] = list(traci.vehicle.getRoute(vehicle_id))
                case _:
                    raise ValueError("(step {0}) Simulation.get_vehicle_vals(): Unrecognised key ('{1}').".format(self.curr_step, data_key))

        if set(data_vals.keys()) != set(data_vals):
            raise ValueError("(step {0}) Simulation.get_vehicle_vals(): Invalid data_keys given (must be from accepted list).".format(self.curr_step))
        if return_val: return list(data_vals.values())[0]
        else: return data_vals
    
    def get_vehicle_data(self, vehicle_id: str, assert_exists: bool = True) -> dict:
        """
        Get data for specified vehicle, updating known_vehicles dict.
        :param vehicle_id: Vehicle ID
        :param assert_exists: If True, error is raised if vehicle does not exist
        :return dict: Vehicle data dictionary, returns None if does not exist in simulation
        """

        if not self.vehicle_exists(vehicle_id):
            if assert_exists: raise KeyError("(step {0}) Simulation.get_vehicle_data(): Unrecognised vehicle ID found ('{1}').".format(self.curr_step, vehicle_id))
            elif not self.suppress_warnings:
                print("(step {0}) (WARNING) Simulation.get_vehicle_data(): Unrecognised vehicle ID given ('{1}').".format(self.curr_step, vehicle_id))
                return None
        
        vehicle_data = self.get_vehicle_vals(vehicle_id, ("type", "speed", "position", "heading", "ts", "length"))

        if vehicle_id not in self.known_vehicles.keys():
            self.known_vehicles[vehicle_id] = {"type":      vehicle_data["type"],
                                               "longitude": vehicle_data["position"][0],
                                               "latitude":  vehicle_data["position"][1],
                                               "speed":     vehicle_data["speed"],
                                               "stopped":   self.vehicle_is_stopped(vehicle_id, vehicle_data["speed"]),
                                               "length":    vehicle_data["length"],
                                               "heading":   vehicle_data["heading"],
                                               "ts":        vehicle_data["ts"],
                                               "altitude":  vehicle_data["position"][2],
                                               "last_seen": self.curr_step
                                              }
        else:
            self.known_vehicles[vehicle_id]["speed"]     = vehicle_data["speed"]
            self.known_vehicles[vehicle_id]["stopped"]   = self.vehicle_is_stopped(vehicle_id, vehicle_data["speed"])
            self.known_vehicles[vehicle_id]["longitude"] = vehicle_data["position"][0]
            self.known_vehicles[vehicle_id]["latitude"]  = vehicle_data["position"][1]
            self.known_vehicles[vehicle_id]["heading"]   = vehicle_data["heading"]
            self.known_vehicles[vehicle_id]["ts"]        = vehicle_data["ts"]
            self.known_vehicles[vehicle_id]["altitude"]  = vehicle_data["position"][2]
            self.known_vehicles[vehicle_id]["last_seen"] = self.curr_step

        vehicle_data = copy(self.known_vehicles[vehicle_id])
        del vehicle_data['last_seen']

        return vehicle_data

    def get_all_vehicle_data(self, types: list = None, assert_all_vtypes: bool = False) -> dict:
        """
        Collects vehicle data from SUMO, by id and/or type (defaults to all vehicles).
        :param types: Type(s) of vehicles to fetch
        :return dict: Stats & vehicle data by id
        """

        all_vehicle_data = {}
        total_vehicle_data = {"no_vehicles": 0, "no_waiting": 0}

        for vehicle_id in self.all_curr_vehicle_ids:

            # Saving known vehicles reduces calls to TraCI by not
            # fetching already known (& unchanging!) data
            if vehicle_id in self.known_vehicles.keys(): vehicle_type = self.known_vehicles[vehicle_id]["type"]
            else: vehicle_type = traci.vehicle.getTypeID(vehicle_id)

            if types is None or (isinstance(types, list) and vehicle_type in types) or (isinstance(types, str) and vehicle_type == types):

                if self.get_individual_vehicle_data:
                    vehicle_data = self.get_vehicle_data(vehicle_id, vehicle_type)
                    all_vehicle_data[vehicle_id] = vehicle_data
                    if vehicle_data["stopped"]: total_vehicle_data["no_waiting"] += 1

                elif self.vehicle_is_stopped(vehicle_id): total_vehicle_data["no_waiting"] += 1

                total_vehicle_data["no_vehicles"] += 1

        return total_vehicle_data, all_vehicle_data
    
    def vehicle_is_stopped(self, vehicle_id: str|int, speed: float|None = None) -> bool:
        """
        Check if vehicle is stopped (is moving at less than 0.1m/s^2).
        :param vehicle_id: Vehicle ID
        :return bool: Denoting whether is stopped
        """

        if speed == None: speed = self.get_vehicle_vals(vehicle_id, "speed")
        if self.units.name in ['IMPERIAL', 'UK'] and speed < 0.223694: return True
        elif self.units.name == 'METRIC' and speed < 0.36: return True
        else: return False
    
    def get_geometry_vals(self, geometry_id: str|int, data_keys: str|list) -> dict|str|int|float|list:
        """
        Get data values for specific edge or lane using a list of data keys, from: (edge or lane) n_vehicles, vehicle_ids, vehicle_speed,
        vehicle_halting, vehicle_occupancy, tt (travel time), emissions, max_speed (avg of lane speeds) | (edge only) street_name,
        n_lanes, lane_ids | (lane only) edge_id, n_links, permissions
        :param edge_id: Either lane or edge ID
        :param data_keys: List of keys or single key
        :return dict: Values by data_key (or single value)
        """
        
        g_name = self.geometry_exists(geometry_id)
        if g_name == "EDGE": g_class = traci.edge
        elif g_name == "LANE": g_class = traci.lane
        else: raise KeyError("(step {0}) Simulation.get_geometry_vals(): Geometry ID '{1}' not found.".format(self.curr_step, geometry_id))

        return_val = False
        if isinstance(data_keys, str):
            data_keys = [data_keys]
            return_val = True
        elif not isinstance(data_keys, (list, tuple)):
            raise TypeError("(step {0}) Simulation.get_geometry_vals(): Invalid data_keys given '{1}' (must be [str|(str)], not '{2}').".format(self.curr_step, data_keys, type(data_keys).__name__))
        
        data_vals = {}
        for data_key in data_keys:
            match data_key:
                case "n_vehicles":
                    data_vals[data_key] = g_class.getLastStepVehicleNumber(geometry_id)
                    continue
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
                    
            if g_name == "EDGE":
                match data_key:
                    case "street_name":
                        data_vals[data_key] = g_class.getStreetName(geometry_id)
                    case "n_lanes":
                        data_vals[data_key] = g_class.getLaneNumber(geometry_id)
                    case "lane_ids":
                        n_lanes = g_class.getLaneNumber(geometry_id)
                        data_vals[data_key] = ["{0}_{1}".format(geometry_id, idx) for idx in range(n_lanes)]
                    case "max_speed":
                        n_lanes = g_class.getLaneNumber(geometry_id)
                        lane_ids = ["{0}_{1}".format(geometry_id, idx) for idx in range(n_lanes)]
                        avg_lane_speed = sum([traci.lane.getMaxSpeed(lane_id) for lane_id in lane_ids])/len(lane_ids)

                        if self.units.name in ['IMPERIAL', 'UK']: data_vals[data_key] = avg_lane_speed * 2.2369362920544
                        else: data_vals[data_key] = avg_lane_speed * 3.6
                    case _:
                        raise ValueError("(step {0}) Simulation.get_geometry_vals(): Unrecognised key ('{1}').".format(self.curr_step, data_key))
            elif g_name == "LANE":
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
                        raise ValueError("(step {0}) Simulation.get_geometry_vals(): Unrecognised key ('{1}').".format(self.curr_step, data_key))
        
        if set(data_vals.keys()) != set(data_vals):
            raise ValueError("(step {0}) Simulation.get_geometry_vals(): Invalid data_keys given (must be from accepted list).".format(self.curr_step))
        if return_val: return list(data_vals.values())[0]
        else: return data_vals

    def set_geometry_vals(self, geometry_id: str|int, **kwargs) -> None:
        """
        Calls the TraCI API to change a edge or lane's state.
        :param geometry_id: Edge or lane ID
        :param max_speed: Set new max speed value [int|float]
        :param allowed: Set allowed vehicle types [(str)], empty list allows all (lane only)
        :param disallowed: Set disallowed vehicle types [(str)] (lane only)
        :param left_lc: Set left lane changing vehicle permission with by vtype list [(str)] (lane only)
        :param right_lc: Set right lane changing vehicle permission with by vtype list [(str)] (lane only)
        """
        
        if geometry_id in self.all_edges:   g_class, g_name = traci.edge, "EDGE"
        elif geometry_id in self.all_lanes: g_class, g_name = traci.lane, "LANE"
        else: raise KeyError("(step {0}) Simulation.set_geometry_vals(): Unrecognised egde or lane ID given ('{1}').".format(self.curr_step, geometry_id))
        
        for command, value in kwargs.items():
            match command:
                case "max_speed":
                    if isinstance(value, (int, float)): 
                        if self.units.name in ['IMPERIAL', 'UK']: g_class.setMaxSpeed(geometry_id, value / 2.2369362920544)
                        else: g_class.setMaxSpeed(geometry_id, value / 3.6)
                    else: raise TypeError("(step {0}) Simulation.set_geometry_vals() ({1}): Invalid speed value '{2}' (must be [int|float], not '{3}').".format(self.curr_step, command, value, type(value).__name__))

                case "allowed":
                    if g_name != "LANE": raise ValueError("(step {0}) Simulation.set_geometry_vals() ({1}): Command is only valid for lanes.".format(self.curr_step, command))
                    if isinstance(value, (list, tuple)):
                        curr_allowed = list(g_class.getAllowed(geometry_id))
                        allowed = tuple(set(curr_allowed + list(value)))
                        g_class.setAllowed(geometry_id, allowed)
                        
                        curr_disallowed = list(g_class.getDisallowed(geometry_id))
                        disallowed = tuple(set(curr_disallowed) - set(value))
                        g_class.setDisallowed(geometry_id, disallowed)
                    else: raise TypeError("(step {0}) Simulation.set_geometry_vals() ({1}): Invalid type list value '{2}' (must be [str], not '{3}').".format(self.curr_step, command, value, type(value).__name__))

                case "disallowed":
                    if g_name != "LANE": raise ValueError("(step {0}) Simulation.set_geometry_vals() ({1}): Command is only valid for lanes.".format(self.curr_step, command))
                    if isinstance(value, (list, tuple)):
                        curr_disallowed = list(g_class.getDisallowed(geometry_id))
                        disallowed = tuple(set(curr_disallowed + list(value)))
                        g_class.setDisallowed(geometry_id, disallowed)
                        
                        curr_allowed = list(g_class.getAllowed(geometry_id))
                        allowed = tuple(set(curr_allowed) - set(value))
                        g_class.setAllowed(geometry_id, allowed)
                    else: raise TypeError("(step {0}) Simulation.set_geometry_vals() ({1}): Invalid type list value '{2}' (must be [str], not '{3}').".format(self.curr_step, command, value, type(value).__name__))

                case "left_lc":
                    if g_name != "LANE": raise ValueError("(step {0}) Simulation.set_geometry_vals() ({1}): Command is only valid for lanes.".format(self.curr_step, command))
                    if isinstance(value, (list, tuple)):
                        g_class.setChangePermissions(geometry_id, value[0], 1)
                    else: raise TypeError("(step {0}) Simulation.set_geometry_vals() ({1}): Invalid type list value '{2}' (must be [str], not '{3}').".format(self.curr_step, command, value, type(value).__name__))
                
                case "right_lc":
                    if g_name != "LANE": raise ValueError("(step {0}) Simulation.set_geometry_vals() ({1}): Command is only valid for lanes.".format(self.curr_step, command))
                    if isinstance(value, (list, tuple)):
                        g_class.setChangePermissions(geometry_id, value[0], -1)
                    else: raise TypeError("(step {0}) Simulation.set_geometry_vals() ({1}): Invalid type list value '{2}' (must be [str], not '{3}').".format(self.curr_step, command, value, type(value).__name__))

                case _:
                    raise ValueError("(step {0}) Simulation.set_vehicle_vals(): Unrecognised command ('{1}').".format(self.curr_step, command))
                
    def get_last_step_geometry_vehicles(self, geometry_ids: str|list, v_types: list|None = None, flatten: bool = False) -> dict|list:
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
    
    def geometry_exists(self, geometry_id: str|int) -> str|None:
        """
        Get geometry type by ID, if exists.
        :param geometry_id: Lane or edge ID
        :return str|None: Geometry type, or None if it does not exist.
        """

        if geometry_id in self.all_edges: return "EDGE"
        elif geometry_id in self.all_lanes: return "LANE"
        else: return None

    def detector_exists(self, detector_id: str) -> str|None:
        """
        Get detector type by ID, if exists.
        :param detector_id: Detector ID
        :return str|None: Detector type, or None if it does not exist.
        """

        if detector_id in self.available_detectors.keys():
            return self.available_detectors[detector_id]["type"]
        else: return None

class TrackedJunction:
    def __init__(self, junc_id: str|int, sim: Simulation, junc_params: dict|str=None) -> None:
        self.id = junc_id
        self.position = traci.junction.getPosition(junc_id)

        self.sim = sim
        self.init_time = sim.curr_step
        self.curr_time = sim.curr_step
        
        self.has_tl = junc_id in sim.all_tls

        if self.has_tl:
            state_str = traci.trafficlight.getRedYellowGreenState(junc_id)
            self.m_len = len(state_str)
            self.states, self.curr_state = [], []        
            self.durations = [[] for _ in range(self.m_len)]
            self.avg_green, self.avg_m_red = 0, 0
            self.avg_m_green, self.avg_m_red = [0 for _ in range(self.m_len)], [0 for _ in range(self.m_len)]

        if junc_params != None:
            junc_params = load_params(junc_params, "Junc.init", self.curr_time, "junc_params")

            self.track_flow = False
            if "flow_params" in junc_params.keys():
                flow_params = junc_params["flow_params"]
                if "inflow_detectors" in flow_params.keys() or "outflow_detectors" in flow_params.keys():
                    if not ("inflow_detectors" in flow_params.keys() and "outflow_detectors" in flow_params.keys()):
                        raise KeyError("(step {0}) TrackedJunction.init(): Both 'inflow_detectors' and 'outflow_detectors' are required parameters to track flow.".format(sim.curr_step))
                    else:

                        for detector_id in flow_params["inflow_detectors"]:
                            if detector_id not in self.sim.available_detectors.keys():
                                raise KeyError("(step {0}) TrackedJunction.init(): Unrecognised detector ID given in inflow_detectors ('{1}').".format(sim.curr_step, detector_id))
                        for detector_id in flow_params["outflow_detectors"]:
                            if detector_id not in self.sim.available_detectors.keys():
                                raise KeyError("(step {0}) TrackedJunction.init(): Unrecognised detector ID given in outflow_detectors ('{1}').".format(sim.curr_step, detector_id))

                        self.inflow_detectors = flow_params["inflow_detectors"]
                        self.outflow_detectors = flow_params["outflow_detectors"]

                    if "flow_vtypes" in flow_params.keys(): self.flow_vtypes = ["all"] + flow_params["flow_vtypes"]
                    else: self.flow_vtypes = ["all"]

                    self.avg_horizon = int(60 / self.sim.step_length) if "avg_horizon" not in flow_params.keys() else int(flow_params["avg_horizon"])
                    
                    self.v_in, self.v_out = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}
                    self.avg_inflow, self.avg_outflow = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}
                    self.inflows, self.outflows = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}

                    self.track_flow = True

            if "meter_params" in junc_params.keys():
                meter_params = junc_params["meter_params"]
                self.is_meter = True
                self.min_rate, self.max_rate = meter_params["min_rate"], meter_params["max_rate"]

                self.metering_rates = []
                self.rate_times = []

                self.measure_spillback = False
                self.measure_queues = False
                self.queue_detector = None
                self.ramp_edges = None

                self.queue_lengths, self.queue_delays = [], []

                if "ramp_edges" in meter_params.keys():
                    self.ramp_edges = meter_params["ramp_edges"]

                    for edge in self.ramp_edges:
                        if edge not in self.sim.all_edges:
                            raise KeyError("(step {0}) TrackedJunction.init(): Edge ID '{1}' not found.".format(self.sim.curr_step, edge))

                    self.measure_spillback, self.spillback_vehs = True, []
                    self.measure_queues, self.queue_detector = True, None
                    self.curr_queuing = set([])

                if not self.measure_queues:
                    if "queue_detector" in meter_params.keys():
                        self.measure_queues, self.queue_detector = True, meter_params["queue_detector"]

                        if self.queue_detector not in self.sim.available_detectors.keys():
                            raise KeyError("(step {0}) TrackedJunction.init(): Unrecognised detector ID given as queue_detector ('{1}').".format(self.sim.curr_step, self.queue_detector))
                        elif self.sim.available_detectors[self.queue_detector]["type"] != "multientryexit":
                            raise AssertionError("(step {0}) TrackedJunction.init(): Only 'multientryexit' detectors can be used to find queue length (not '{1}').".format(self.sim.curr_step, self.sim.available_detectors[self.queue_detector]["type"]))

                if "init_rate" in meter_params.keys(): self.sim.set_tl_metering_rate(self.id, meter_params["init_rate"])
                else: self.sim.set_tl_metering_rate(self.id, self.max_rate)

            else: self.is_meter = False
    
    def __str__(self): return "<TrackedJunction: '{0}'>".format(self.id)

    def get_curr_data(self) -> dict:
        """
        Returns the current state of junction data as a dictionary.
        :return dict: TrackedJunction data dictionary
        """

        junc_dict = {"position": self.position, "init_time": self.init_time, "curr_time": self.curr_time}
        
        if self.has_tl: junc_dict["tl"] = {"m_len": self.m_len, "avg_green": self.avg_green, "avg_red": self.avg_red,
                                           "avg_m_green": self.avg_m_green, "avg_m_red": self.avg_m_red, "m_phases": self.durations}

        if self.track_flow:
            junc_dict["flows"] = {"inflow_detectors": self.inflow_detectors, "outflow_detectors": self.outflow_detectors,
                                 "avg_inflows": self.avg_inflow, "avg_outflows": self.avg_outflow,
                                 "all_inflows": self.inflows, "all_outflows": self.outflows}
            
        if self.is_meter:
            junc_dict["meter"] = {"metering_rates": self.metering_rates, "rate_times": self.rate_times}
            
            if self.measure_queues:
                junc_dict["meter"]["queue_lengths"] = self.queue_lengths
                junc_dict["meter"]["queue_delays"] = self.queue_delays

            if self.measure_spillback:
                junc_dict["meter"]["spillback_vehs"] = self.spillback_vehs

            junc_dict["meter"]["min_rate"] = self.min_rate
            junc_dict["meter"]["max_rate"] = self.max_rate

        return junc_dict

    def reset(self) -> None:
        """
        Resets junction data collection.
        """
        
        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        if self.has_tl:
            self.states = []
            self.durations = [[] for _ in range(self.m_len)]
            self.avg_green, self.avg_m_red = 0, 0
            self.avg_m_green, self.avg_m_red = [0 for _ in range(self.m_len)], [0 for _ in range(self.m_len)]

        if self.track_flow:
            self.v_in, self.v_out = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}
            self.avg_inflow, self.avg_outflow = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}
            self.inflows, self.outflows = {vtype: [] for vtype in self.flow_vtypes}, {vtype: [] for vtype in self.flow_vtypes}

        if self.is_meter:
            self.metering_rates = []
            self.rate_times = []

            if self.measure_queues:
                self.queue_lengths = []
                self.queue_delays = []

            if self.measure_spillback:
                self.spillback_vehs = []

    def update(self) -> None:
        """
        Update junction flow and TL data for the current time step.
        """

        self.curr_time = self.sim.curr_step
        
        if self.has_tl:
            curr_state = traci.trafficlight.getRedYellowGreenState(self.id)
            colours = [*curr_state]
            for idx, mc in enumerate(colours):
                
                if len(self.durations[idx]) == 0 or mc.upper() != self.durations[idx][-1][0]:
                    self.durations[idx].append([mc.upper(), 1])
                elif mc.upper() == self.durations[idx][-1][0]:
                    self.durations[idx][-1][1] = self.durations[idx][-1][1] + 1

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
                self.avg_inflow[vtype].append((sum(self.inflows[vtype][-self.avg_horizon:]) / len(self.inflows[vtype][-self.avg_horizon:])) / self.sim.step_length)

            for vtype in self.flow_vtypes:
                new_v_out = [v_id for v_id in step_v_out if v_id not in self.v_out[vtype] and (vtype == "all" or v_id.startswith(vtype))]
                self.v_out[vtype] += new_v_out
                self.outflows[vtype].append(len(new_v_out))
                self.avg_outflow[vtype].append((sum(self.outflows[vtype][-self.avg_horizon:]) / len(self.outflows[vtype][-self.avg_horizon:])) / self.sim.step_length)

        if self.measure_queues:

            if self.ramp_edges != None:
                queuing_vehicles = self.sim.get_last_step_geometry_vehicles(self.ramp_edges, flatten=True)
                self.queue_lengths.append(len(queuing_vehicles))

                num_stopped = len([veh_id for veh_id in queuing_vehicles if self.sim.vehicle_is_stopped(veh_id)])
                self.queue_delays.append(num_stopped * self.sim.step_length)
            
            elif self.queue_detector != None:
                queuing_vehicles = self.sim.get_last_step_detector_vehicles(self.queue_detector, flatten=True)
                self.queue_lengths.append(len(queuing_vehicles))

                num_stopped = len([veh_id for veh_id in queuing_vehicles if self.sim.vehicle_is_stopped(veh_id)])
                self.queue_delays.append(num_stopped * self.sim.step_length)

            else: raise AssertionError("(step {0}) TrackedJunction.update(): Cannot update queue length (no detector or entry/exit edges)".format(self.sim.curr_step))

        if self.measure_spillback:
            spillback_vehs, all_waiting_vehs = 0, 0
            all_loaded_vehicles = self.sim.all_loaded_vehicle_ids
            for veh_id in all_loaded_vehicles:
                if not self.sim.vehicle_departed(veh_id):
                    if traci.vehicle.getRoute(veh_id)[0] == self.ramp_edges[0]:
                        spillback_vehs += 1
                    all_waiting_vehs += 1

            self.spillback_vehs.append(spillback_vehs)

class TrackedEdge:
    def __init__(self, edge_id: str, simulation: Simulation) -> None:
        self.id = edge_id
        self.sim = simulation

        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        net_file = traci.simulation.getOption("net-file")
        net = sumolib.net.readNet(net_file)
        edge = net.getEdge(edge_id)

        self.linestring = edge.getShape()
        self.line = LineString(self.linestring)
        self.length = self.line.length
        
        if self.sim.units in ['IMPERIAL']: self.length *= 0.0006213712

        self.step_vehicles = []

    def __str__(self): return "<TrackedEdge: '{0}'>".format(self.id)

    def get_curr_data(self) -> dict:
        """
        Returns the current state of edge data as a dictionary.
        :return dict: TrackedEdge data dictionary
        """
        edge_dict = {"linestring": self.linestring,
                     "length": self.length,
                     "step_vehicles": self.step_vehicles,
                     "init_time": self.init_time,
                     "curr_time": self.curr_time}
        
        return edge_dict

    def reset(self) -> None:
        """
        Resets edge data collection.
        """

        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        self.step_vehicles = []

    def update(self) -> None:
        
        self.curr_time = self.sim.curr_step
        last_step_vehs = self.sim.get_last_step_geometry_vehicles(self.id, flatten=True)

        veh_data = []
        for veh_id in last_step_vehs:
            speed = self.sim.get_vehicle_vals(veh_id, "speed")
            coors = self.sim.get_vehicle_vals(veh_id, "position")[:2]
            x = self.get_distance_on_road(coors)
            if self.sim.units in ['IMPERIAL']: x *= 0.0006213712
            veh_data.append((x, speed))

        self.step_vehicles.append(veh_data)
            
    def get_distance_on_road(self, veh_coors):
        line = LineString(self.linestring)
        p = Point(veh_coors)
        p2 = line.interpolate(line.project(p))
        x_val = line.line_locate_point(p2, False)
        x_pct = x_val/line.length
        return x_pct