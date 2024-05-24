import os, sys, io, traci, sumolib, json, math
from tqdm import tqdm
from copy import copy, deepcopy
from events import EventScheduler, Event
from controllers import VSLController, RGController
from shapely.geometry import LineString, Point
from utils import *

class Simulation:
    def __init__(self, scenario_name: str|None = None, scenario_desc: str|None = None) -> None:
        """
        :param scenario_name: Scenario label saved to simulation object (defaults to name of '.sumocfg')
        :param scenario_desc: Simulation scenario description, saved along with all files
        """

        path_tools = os.path.join(os.environ.get("SUMO_HOME"), 'tools')

        if path_tools in sys.path: pass
        else: sys.path.append(path_tools)

        self.curr_step = 0
        self.step_length = None
        self.units = Units(1)
        self.scenario_name = scenario_name
        self.scenario_desc = scenario_desc
        self._seed = None
        self._running = False
        self._gui = False

        self.all_data = None

        self.track_juncs = False
        self.tracked_juncs = {}
        self._junc_phases = None
        self._all_juncs = []
        self._all_tls = []

        self.controllers = {}
        self._scheduler = None

        self.tracked_edges = {}
        self.available_detectors = {}

        self._all_edges = None
        self._all_lanes = None
        self._all_routes = None

        self._get_individual_vehicle_data = True
        self._all_curr_vehicle_ids = set([])
        self._all_loaded_vehicle_ids = set([])
        self._known_vehicles = {}

    def __str__(self):
        if self.scenario_name != None:
            return "<Simulation: '{0}'>".format(self.scenario_name)
        else: return "<Simulation>"

    def __name__(self): return "Simulation"

    def __dict__(self): return {} if self.all_data == None else self.all_data

    def start(self, config_file: str|None = None, net_file: str|None = None, route_file: str|None = None, add_file: str|None = None, cmd_options: list|None = None, units: str|int = 1, get_individual_vehicle_data: bool = True, suppress_warnings: bool = False, ignore_TraCI_err: bool = False, seed: str|None = None, gui: bool = False) -> None:
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
        :param seed:        Either int to be used as seed, or random.random()/random.randint(), where a random seed is used
        :param gui:         Bool denoting whether to run GUI
        """

        self._gui = gui
        sumoCMD = ["sumo-gui"] if self._gui else ["sumo"]

        if config_file == net_file == None:
            desc = "Either config or network file required."
            raise_error(ValueError, desc, self.curr_step)
        
        if config_file != None:
            if config_file.endswith(".sumocfg"):
                sumoCMD += ["-c", config_file]
                if self.scenario_name == None: self.scenario_name = get_scenario_name(config_file)
            else:
                desc = "Invalid config file extension."
                raise_error(ValueError, desc, self.curr_step)
        else:
            sumoCMD += ["-n", net_file]
            if self.scenario_name == None: self.scenario_name = get_scenario_name(net_file)
            if route_file != None: sumoCMD += ["-r", route_file]
            if add_file != None: sumoCMD += ["-a", add_file]
        
        if cmd_options != None: sumoCMD += cmd_options

        if isinstance(seed, str): sumoCMD += ["--seed", seed]
        elif seed != None:
            desc = "Invalid seed type (must be 'str', not '{0}').".format(type(seed).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        self._seed = seed

        if isinstance(units, str):
            if units.upper() in ["METRIC", "IMPERIAL", "UK"]:
                self.units = Units[["METRIC", "IMPERIAL", "UK"].index(units)]
            else:
                desc = "Invalid simulation units '{0}' (must be ['METRIC'|'IMPERIAL'|'UK']).".format(units)
                raise_error(ValueError, desc, self.curr_step)

        elif units in [1, 2, 3]: self.units = Units(units)
        else:
            desc = "Invalid simulation units '{0}' (must be ['METRIC'|'IMPERIAL'|'UK']).".format(units)
            raise_error(ValueError, desc, self.curr_step)

        traci.start(sumoCMD)
        self._running = True

        self.step_length = float(traci.simulation.getOption("step-length"))

        if self._junc_phases != None: self._update_lights()
        self._time_val = traci.simulation.getTime()

        for detector_id in list(traci.multientryexit.getIDList()):
            self.available_detectors[detector_id] = {'type': 'multientryexit', 'position': {'entry_lanes': traci.multientryexit.getEntryLanes(detector_id),
                                                                                            'exit_lanes': traci.multientryexit.getExitLanes(detector_id),
                                                                                            'entry_positions': traci.multientryexit.getEntryPositions(detector_id),
                                                                                            'exit_positions': traci.multientryexit.getExitPositions(detector_id)}}
            
        for detector_id in list(traci.inductionloop.getIDList()):
            self.available_detectors[detector_id] = {'type': 'inductionloop', 'position': {'lane_id': traci.inductionloop.getLaneID(detector_id), 'position': traci.inductionloop.getPosition(detector_id)}}

        self._all_juncs = list(traci.junction.getIDList())
        self._all_tls = list(traci.trafficlight.getIDList())
        self._all_edges = traci.edge.getIDList()
        self._all_lanes = traci.lane.getIDList()
        self._all_routes = traci.route.getIDList()

        self._get_individual_vehicle_data = get_individual_vehicle_data

        self._ignore_TraCI_err = ignore_TraCI_err
        self._suppress_warnings = suppress_warnings

        self._sim_start_time = get_time_str()
        
    def add_tracked_junctions(self, juncs: str|list|dict|None = None) -> None:
        """
        Initalise junctions and start tracking states and flows.
        :param juncs: Either junc_id|list of junc_ids, or dict containing juncs parameters. Defaults to all junctions with traffic lights.
        """

        self.track_juncs = True

        if juncs == None: # If none given, track all junctions with traffic lights
            track_list, junc_params = self._all_tls, None
        else:
            
            if isinstance(juncs, dict):
                junc_ids, junc_params = list(juncs.keys()), juncs
            elif isinstance(juncs, (list, tuple)):
                junc_ids, junc_params = juncs, None
            elif isinstance(juncs, str):
                junc_ids, junc_params = [juncs], None
            else:
                desc = "Invalid junc_params (must be [str|list|dict], not '{0}').".format(type(juncs).__name__)
                raise_error(TypeError, desc, self.curr_step)

            if len(set(self._all_juncs).intersection(set(junc_ids))) != len(junc_ids):
                desc = "Junction ID(s) not found ('{0}').".format("', '".join(set(junc_ids) - set(self._all_juncs)))
                raise_error(KeyError, desc, self.curr_step)
            else: track_list = junc_ids

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

        self._sim_start_time = get_time_str()
        self.all_data = None

    def is_running(self, close: bool=True) -> bool:
        """
        Returns whether the simulation is running.
        :param close: If True, end Simulation
        :return bool: Denotes if the simulation is running
        """
        if not self._running: return self._running

        elif traci.simulation.getMinExpectedNumber() == 0:
            if close: self.end()
            print('Ended simulation: no vehicles remaining.')
            return False
        
        return True

    def end(self) -> None:
        """
        Ends the simulation.
        """

        try:
            traci.close()
        except traci.exceptions.FatalTraCIError:
            raise_warning("TraCI is not connected.", self.curr_step)
        self._running = False

    def save_data(self, filename: str|None = None, overwrite: bool = True, json_indent: int|None = 4) -> None:
        """
        Save all vehicle, detector and junction data in a JSON file.
        :param filename:  Output filepath (defaults to ./'scenario_name'.json)
        :param overwrite: Prevent previous outputs being overwritten
        """

        if filename == None: filename = self.scenario_name
        if not filename.endswith(".json"): filename += ".json"

        if os.path.exists(filename) and overwrite:
            if not self._suppress_warnings: raise_warning("File '{0}' already exists and will be overwritten.".format(filename), self.curr_step)
        elif os.path.exists(filename) and not overwrite:
            desc = "File '{0}' already exists and cannot be overwritten.".format(filename)
            raise_error(FileExistsError, desc, self.curr_step)

        if self.all_data != None:
            if self._scheduler != None: self.all_data["data"]["events"] = self._scheduler.__dict__()
            with open(filename, "w") as fp:
                json.dump(self.all_data, fp, indent=json_indent)
        else:
            desc = "No data to save as a simulation has not been run."
            raise_error(IndexError, desc, self.curr_step)

    def add_tracked_edges(self, edge_ids=None):
        """
        Initalise edges and start collecting data.
        :param edge_ids: List of edge_ids, defaults to all
        """
        if edge_ids == None: edge_ids = self._all_edges
        for edge_id in edge_ids:
            if self.geometry_exists(edge_id) == None:
                desc = "Geometry ID '{0}' not found.".format(edge_id)
                raise_error(KeyError, desc, self.curr_step)
            self.tracked_edges[edge_id] = TrackedEdge(edge_id, self)

    def add_events(self, event_params: Event|list|dict|str) -> None:
        """
        Add events and event scheduler.
        :param event_parms: Event parameters [Event|[Event]|dict|filepath]
        """
        if self._scheduler == None:
            self._scheduler = EventScheduler(self)
        self._scheduler.add_events(event_params)

    def add_controllers(self, controller_params: str|dict) -> dict:
        """
        Add controllers from parameters in a dictionary/JSON file.
        :param controller_params: Controller parameters dictionary or filepath
        """

        controller_params = load_params(controller_params, "controller_params", self.curr_step)

        for c_id, c_params  in controller_params.items():
            if isinstance(c_params, (RGController, VSLController)):
                self.controllers[c_id] = c_params
            elif isinstance(c_params, dict):
                if 'type' not in c_params.keys():
                    desc = "No type given (must be [1 (RG)|2 (VSL)])."
                    raise_error(KeyError, desc, self.curr_step)
                if c_params['type'] in [1, 2, "VSL", "RG"]:
                    
                    if c_params['type'] in [1, "VSL"]: controller = RGController(c_id, c_params, self)
                    elif c_params['type'] in [2, "RG"]: controller = RGController(c_id, c_params, self)
                    
                    self.controllers[c_id] = controller

                else:
                    desc = "Invalid controller type (must be [1 (RG)|2 (VSL)])."
                    raise_error(ValueError, desc, self.curr_step)
            else:
                desc = "Invalid parameters type in dictionary (must be [dict|RGController|VSLController], not '{0}').".format(type(c_params).__name__)
                raise_error(TypeError, desc, self.curr_step)

        return self.controllers

    def step_through(self, n_steps: int = 1, end_step: int|None = None, sim_dur: int|None = None, detector_list: list|None = None, vTypes: list|None = None, keep_data: bool = True, append_data: bool = True, cumulative_data: bool = False, pbar: tqdm|None = None) -> dict:
        """
        Step through simulation from the current time until end_step, aggregating data during this period.
        :param n_steps:         Perform n steps of the simulation (defaults to 1)
        :param end_step:        End point for stepping through simulation (given instead of end_step)
        :param sim_dur:         Simulation duration
        :param detector_list:   List of detector IDs to collect data from (defaults to all)
        :param vTypes:          Vehicle type(s) to collect data of (list of types or string, defaults to all)
        :param keep_data:       Denotes whether to store data collected during this run (defaults to True)
        :param append_data:     Denotes whether to append simulation data to that of previous runs (defaults to True)
        :param cumulative_data: Denotes whether to get cumulative veh count and TTS values
        :param pbar:            tqdm progress bar updated by 1 with each step, used to 
        :return dict:           All data collected through the time period, separated by detector
        """

        if not self.is_running(): return

        if append_data == True: prev_data = self.all_data
        else: prev_data = None

        if detector_list == None: detector_list = list(self.available_detectors.keys())
        start_time = self.curr_step

        if end_step == None and n_steps != None: end_step = self.curr_step + n_steps
        elif end_step == None and sim_dur != None: end_step = self.curr_step + (sim_dur / self.step_length)
        elif end_step == None: 
            desc = "No time value given."
            raise_error(ValueError, desc, self.curr_step)

        if prev_data == None:
            prev_steps, all_data = 0, {"scenario_name": self.scenario_name, "scenario_desc": "", "data": {}, "start": start_time, "end": self.curr_step, "step_len": self.step_length, "units": self.units.name, "seed": 0, "sim_start": self._sim_start_time, "sim_end": get_time_str()}
            
            if self.scenario_desc == None: del all_data["scenario_desc"]
            else: all_data["scenario_desc"] = self.scenario_desc

            if self._seed == None: del all_data["seed"]
            else: all_data["seed"] = self._seed

            if len(self.available_detectors) > 0: all_data["data"]["detector"] = {}
            if self.track_juncs: all_data["data"]["junctions"] = {}
            if len(self.tracked_edges) > 0: all_data["data"]["edges"] = {}
            if len(self.controllers) > 0: all_data["data"]["controllers"] = {}
            all_data["data"]["vehicle"] = {}
            if self._get_individual_vehicle_data: all_data["data"]["all_vehicles"] = []
            if self._scheduler != None: all_data["data"]["events"] = {}
        else: 
            prev_steps = set([len(data_arr) for data_arr in prev_data["data"]["vehicle"].values()] +
                            [len(detector_data["speeds"]) for detector_data in prev_data["data"]["detector"].values()] +
                            [len(detector_data["veh_counts"]) for detector_data in prev_data["data"]["detector"].values()])
            
            if len(prev_steps) != 1:
                desc = "Invalid prev_data (different length arrays)."
                raise_error(ValueError, desc, self.curr_step)
            else:
                prev_steps = prev_steps.pop()
                all_data = prev_data

        if not isinstance(pbar, tqdm):
            if not self._gui and n_steps > 1: pbar = tqdm(desc="Running sim (step {0}, {1} vehs)".format(self.curr_step, len(self._all_curr_vehicle_ids)), total=end_step - self.curr_step)

        while self.curr_step < end_step:

            try: last_step_data, all_v_data = self.step(detector_list, vTypes)
            except traci.exceptions.FatalTraCIError:
                self._running = False
                if self._ignore_TraCI_err:
                    if not self._suppress_warnings: raise_warning("Fatal TraCI connection error occured.", self.curr_step)
                    break
                else:
                    desc = "Fatal TraCI connection error occured."
                    raise_error(traci.exceptions.FatalTraCIError, desc, self.curr_step)

            if self._get_individual_vehicle_data: all_data["data"]["all_vehicles"].append(all_v_data)
            for controller in self.controllers.values(): controller.update()
            for edge in self.tracked_edges.values(): edge.update()

            if len(all_data["data"]["detector"]) == 0:
                for detector_id in detector_list:
                    all_data["data"]["detector"][detector_id] = self.available_detectors[detector_id]
                    all_data["data"]["detector"][detector_id].update({"speeds": [], "veh_counts": [], "veh_ids": [], "occupancies": []})
                all_data["data"]["vehicle"] = {"no_vehicles": [], "tts": [], "delay": []}

            for detector_id in last_step_data["detector"].keys():
                if detector_id not in all_data["data"]["detector"].keys():
                    desc = "Unrecognised detector ID found ('{0}').".format(detector_id)
                    raise_error(KeyError, desc, self.curr_step)
                for data_key, data_val in last_step_data["detector"][detector_id].items():
                    all_data["data"]["detector"][detector_id][data_key].append(data_val)

            for data_key, data_val in last_step_data["vehicle"].items():
                all_data["data"]["vehicle"][data_key].append(data_val)

            if isinstance(pbar, tqdm):
                pbar.update(1)
                pbar.set_description("Running sim (step {0}, {1} vehs)".format(self.curr_step, len(self._all_curr_vehicle_ids)))

        if cumulative_data:
            for detector_data in all_data["data"]["detector"].values():
                detector_data["veh_counts"] = get_cumulative_arr(detector_data["veh_counts"], prev_steps)
            all_data["data"]["vehicle"]["no_vehicles"] = get_cumulative_arr(all_data["data"]["vehicle"]["no_vehicles"], prev_steps)
            all_data["data"]["vehicle"]["tts"] = get_cumulative_arr(all_data["data"]["vehicle"]["tts"], prev_steps)

        all_data["end"] = self.curr_step
        all_data["sim_end"] = get_time_str()
        if self.track_juncs: all_data["data"]["junctions"] = last_step_data["junctions"]
        if self._scheduler != None: all_data["data"]["events"] = self._scheduler.__dict__()
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
        time_diff = traci.simulation.getTime() - self._time_val
        self._time_val = traci.simulation.getTime()

        self._all_curr_vehicle_ids = set(traci.vehicle.getIDList())
        self._all_loaded_vehicle_ids = set(traci.vehicle.getLoadedIDList())
        self._all_to_depart_vehicle_ids = self._all_loaded_vehicle_ids - self._all_curr_vehicle_ids

        if self._junc_phases != None:
            update_junc_lights = []
            for junction_id, phases in self._junc_phases.items():
                phases["curr_time"] += time_diff
                if phases["curr_time"] >= phases["cycle_len"]:
                    phases["curr_time"] = 0
                    phases["curr_phase"] = 0
                    update_junc_lights.append(junction_id)

                # Change to a while loop, updating phase until correct is found? Would then allow for phases with dur less than the step len
                elif phases["curr_time"] >= sum(phases["times"][:phases["curr_phase"] + 1]):
                    phases["curr_phase"] += 1
                    update_junc_lights.append(junction_id)

            self._update_lights(update_junc_lights)

        if self._scheduler != None: self._scheduler.update_events()

        if detector_list == None: detector_list = list(self.available_detectors.keys())
        for detector_id in detector_list:
            data["detector"][detector_id] = {}
            if detector_id not in self.available_detectors.keys():
                desc = "Unrecognised detector ID found ('{0}').".format(detector_id)
                raise_error(KeyError, desc, self.curr_step)
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
                if not self._suppress_warnings: raise_warning("Unknown detector type '{0}'.".format(self.available_detectors[detector_id]["type"]), self.curr_step)

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
            
            if detector_id not in self.available_detectors.keys():
                desc = "Detector ID '{0}' not found.".format(detector_id)
                raise_error(KeyError, desc, self.curr_step)
            detector_type = self.available_detectors[detector_id]["type"]

            if detector_type == "inductionloop":
                detected_vehicles = list(traci.inductionloop.getLastStepVehicleIDs(detector_id))
            elif detector_type == "multientryexit":
                detected_vehicles = list(traci.multientryexit.getLastStepVehicleIDs(detector_id))
            else:
                desc = "Unknown detector type '{0}'".format(detector_type)
                raise_error(KeyError, desc, self.curr_step)
            
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
            desc = "Detector with ID '{0}' not found.".format(detector_id)
            raise_error(KeyError, desc, self.curr_step)
        else:
            detector_type = self.available_detectors[detector_id]["type"]

            match detector_type:
                case "multientryexit": d_class = traci.multientryexit
                case "inductionloop": d_class = traci.inductionloop
                case "_":
                    desc = "Only 'multientryexit' and 'inductionloop' detectors are currently supported (not '{0}').".format(detector_type)
                    raise_error(ValueError, desc, self.curr_step)
            
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
                        desc = "Unrecognised data key ('{0}').".format(data_key)
                        raise_error(KeyError, desc, self.curr_step)
                    
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

        if self.all_data == None:
            desc = "No detector data as the simulation has not been run or data has been reset."
            raise_error(IndexError, desc, self.curr_step)
        elif detector_id not in self.available_detectors.keys() or detector_id not in self.all_data["data"]["detector"].keys():
            desc = "Detector with ID '{0}' not found.".format(detector_id)
            raise_error(KeyError, desc, self.curr_step)

        detector_data = {}
        if not isinstance(data_keys, (list, tuple)): data_keys = [data_keys]
        for data_key in data_keys:
            if data_key in ["veh_counts", "speeds", "occupancies"]:
                if data_key in self.all_data["data"]["detector"][detector_id].keys():
                    data_arr = self.all_data["data"]["detector"][detector_id][data_key][-n_steps:]
                    if avg_vals: data_arr = sum(data_arr) / len(data_arr)
                else:
                    desc = "Detector '{0}' of type '{1}' does not collect '{2}' data.".format(detector_id, self.all_data["data"]["detector"][detector_id]["type"], data_key)
                    raise_error(KeyError, desc, self.curr_step)
            else:
                desc = "Unrecognised data key ('{0}').".format(data_key)
                raise_error(KeyError, desc, self.curr_step)
            detector_data[data_key] = data_arr

        if len(data_keys) == 1: return detector_data[data_keys[0]]
        else: return detector_data

    def set_phases(self, new_junc_phases: dict, start_phase: int = 0, overwrite: bool = True) -> None:
        """
        Sets the phases for the simulation, starting at the next simulation step.
        :param new_junc_phases: Junction phases and times dictionary
        :param start_phase: Phase number to start at, defaults to 0
        :param overwrite: If true, the junc_phases dict is overwitten with new_junc_phases. If false, only specific junctions are overwritten.
        """

        # junc_phases = {junction_0: {phases: [phase_0, phase_1], times: [time_0, time_1]}, ...}
        # where phases are light strings and times are phase length

        if overwrite or self._junc_phases == None:
            self._junc_phases = new_junc_phases
        else:
            for junc_id, new_phases in new_junc_phases.items():
                self._junc_phases[junc_id] = deepcopy(new_phases)

        for junc_id in new_junc_phases.keys():

            if junc_id not in list(traci.trafficlight.getIDList()):
                desc = "Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(junc_id)
                raise_error(KeyError, desc, self.curr_step)

            junc_phase = self._junc_phases[junc_id]

            for t in junc_phase["times"]:
                if t < self.step_length:
                    desc = "Invalid phase duration (phase_dur ({0}) < resolution ({1}))\n.  - {2}".format(t, self.step_length, junc_phase)
                    raise_error(ValueError, desc, self.curr_step)

            if "curr_phase" not in junc_phase.keys(): junc_phase["curr_phase"] = start_phase
            if junc_phase["curr_phase"] > len(junc_phase["phases"]): junc_phase["curr_phase"] -= len(junc_phase["phases"])

            junc_phase["curr_time"] = sum(junc_phase["times"][:junc_phase["curr_phase"]])
            junc_phase["cycle_len"] = sum(junc_phase["times"])

        self._update_lights(new_junc_phases.keys())

    def set_tl_colour(self, junction_id: str|int, colour_str: str) -> None:
        """
        Sets a junction to a set colour/phase string for an indefinite amount of time. Can be used when tracking phases separately.
        :param junction_id: Junction ID
        :param colour_str: Colour character ([r|y|g|-]) or phase string ([r|y|g|-]+)
        """
        
        if junction_id not in list(traci.trafficlight.getIDList()):
            desc = "Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(junction_id)
            raise_error(KeyError, desc, self.curr_step)
        else:
            if junction_id in self.tracked_juncs.keys():
                m_len = self.tracked_juncs[junction_id].m_len
            else:
                state_str = traci.trafficlight.getRedYellowGreenState(junction_id)
                m_len = len(state_str)

        if not isinstance(colour_str, str):
            desc = "Invalid colour_str (must be 'str', not '{0}').".format(type(colour_str).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        if len(colour_str) == 1:
            junc_phases = {junction_id: {"phases": [colour_str*m_len], "times": [math.inf]}}
        elif len(colour_str) == m_len:
            junc_phases = {junction_id: {"phases": [colour_str], "times": [math.inf]}}
        else:
            desc = "Invalid colour_str (must be char or len(str) == junction movements length)."
            raise_error(ValueError, desc, self.curr_step)
        
        self.set_phases(junc_phases, overwrite=False)

    def set_tl_metering_rate(self, rm_id: str|int, metering_rate: int|float, g_time: int|float = 1, y_time: int|float = 1, min_red: int|float = 1, control_interval: int = 60) -> dict:
        """
        Set ramp metering rate of a meter at a junction. Uses a one-car-per-green policy with a default
        1s green and yellow time, with red phase duration changed to set flow. All phase durations must
        be larger than the simulation step length.
        :param rm_id:            Ramp meter (junction) ID
        :param metering_rate:    On-ramp inflow in veh/hr (from all lanes)
        :param g_time:           Green phase duration (s), defaults to 1
        :param y_time:           Yellow phase duration (s), defaults to 1
        :param min_red:          Minimum red phase duration (s), defaults to 1
        :param control_interval: Ramp meter control interval (s) (control interval % cycle length = 0)
        :return dict:            Resulting phase dictionary
        """
        
        if min([g_time, y_time, min_red]) <= self.step_length:
            desc = "Green ({0}), yellow ({1}) and minimum red ({2}) times must all be greater than sim step length ({3}).".format(g_time, y_time, min_red, self.step_length)

        if rm_id not in list(traci.trafficlight.getIDList()):
            desc = "Junction with ID '{0}' does not exist, or it does not have a traffic light.".format(rm_id)
            raise_error(KeyError, desc, self.curr_step)
        else:
            if rm_id in self.tracked_juncs.keys():
                m_len = self.tracked_juncs[rm_id].m_len
            else:
                state_str = traci.trafficlight.getRedYellowGreenState(rm_id)
                m_len = len(state_str)

        if self.track_juncs and rm_id in self.tracked_juncs.keys():
            self.tracked_juncs[rm_id].metering_rates.append(metering_rate)
            self.tracked_juncs[rm_id].rate_times.append(self.curr_step)

        max_flow = (3600 / (g_time + y_time + min_red)) * m_len
        
        # With one lane and a g_time, y_time and min_red of 1s, the meter cannot physically release
        # more than 1200 veh/hr without reducing minimum red. So, to stop the meter continually
        # flipping from g-y-r, the meter is set to green for the whole control interval.

        # This maximum flow is increased with 2 (or more) lanes as, even with 1s green time, this essentially
        # becomes a two-car-per-green policy, and so the maximum flow is doubled.
        if metering_rate > max_flow:
            phases_dict = {"phases": ["G"*m_len], "times": [control_interval/self.step_length]}
        elif metering_rate == 0:
            phases_dict = {"phases": ["r"*m_len], "times": [control_interval/self.step_length]}
        elif metering_rate < 0:
            desc = "Metering rate must be greater than 0 (set to '{0}').".format(metering_rate)
            raise_error(ValueError, desc, self.curr_step)
        else:

            # vehs_per_ci = vehicles released/control interval/lane
            vehs_per_ci = ((metering_rate * control_interval) / 3600) / m_len
            
            # red time calculated with the number of cycles per control interval, minus g + y time
            cycle_length = control_interval / vehs_per_ci
            red_time = cycle_length - g_time - y_time

            # Control interval should be divisible by resulting cycle length as below!
            # sum(phase["times"])/self.step_length * vehs_per_ci == control_interval / self.step_length
            phases_dict = {"phases": ["G"*m_len, "y"*m_len, "r"*m_len],
                        "times":  [g_time / self.step_length, y_time / self.step_length, red_time / self.step_length]}
            
        self.set_phases({rm_id: phases_dict}, overwrite=False)
        return phases_dict

    def change_phase(self, junction_id: str|int, phase_no: int) -> None:
        """
        Change to a different phase at the specified junction_id.
        :param junction_id: Junction ID
        :param phase_no: Phase number
        """
        
        if 0 < phase_no < len(self._junc_phases[junction_id]["phases"]):
            self._junc_phases[junction_id]["curr_phase"] = phase_no
            self._junc_phases[junction_id]["curr_time"] = sum(self.junc_phase["times"][:phase_no])

            self._update_lights(junction_id)

        else:
            desc = "Invalid phase number '{0}' (must be [0-{1}]).".format(phase_no, len(self._junc_phases[junction_id]["phases"]))
            raise_error(ValueError, desc, self.curr_step)

    def _update_lights(self, junction_ids: list|str = None) -> None:
        """
        Update light settings for given junctions.
        :param junction_ids: Junction ID, or list of IDs (defaults to all)
        """

        if junction_ids is None: junction_ids = self._junc_phases.keys()
        elif isinstance(junction_ids, str): junction_ids = [junction_ids]

        for junction_id in junction_ids:
            curr_setting = traci.trafficlight.getRedYellowGreenState(junction_id)
            new_phase = self._junc_phases[junction_id]["phases"][self._junc_phases[junction_id]["curr_phase"]].lower()
            if '-' in new_phase:
                new_phase = new_phase.split()
                new_phase = "".join([new_phase[i] if new_phase[i] != '-' else curr_setting[i] for i in range(len(new_phase))])
            traci.trafficlight.setRedYellowGreenState(junction_id, new_phase)

    def vehicle_exists(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle exists in the network and has departed.
        :return bool: True if ID in list of current vehicle IDs 
        """
        return vehicle_id in self._all_curr_vehicle_ids
    
    def vehicle_loaded(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle is loaded (may not have departed).
        :return bool: True if ID in list of loaded vehicle IDs
        """
        return vehicle_id in self._all_loaded_vehicle_ids
    
    def vehicle_departed(self, vehicle_id: str) -> bool:
        """
        Tests if a vehicle is loaded but has not departed yet.
        :return bool: True if vehicle has not departed yet
        """
        if not self.vehicle_loaded(vehicle_id):
            desc = "Vehicle with ID '{0}' has not been loaded.".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)
        return vehicle_id in self._all_to_depart_vehicle_ids
    
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
            desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)
        
        for command, value in kwargs.items():
            match command:
                case "highlight":
                    if value != None:
                        colour = value
                        if isinstance(colour, str):
                            if "#" in colour: colour = colour.lstrip("#")
                            if len(colour) != 6:
                                desc = "({0}): '{1}' is not a valid hex colour.".format(command, colour)
                                raise_error(ValueError, desc, self.curr_step)
                            colour = tuple(int(colour[i:i+2], 16) for i in (0, 2, 4))
                        elif not isinstance(colour, (list, tuple)):
                            desc = "({0}): Invalid colour (must be [str|list|tuple], not '{1}').".format(command, type(colour).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                        elif len(colour) not in [3, 4] or all(x > 255 for x in colour) or all(x < 0 for x in colour):
                            desc = "({0}): '{1}' is not a valid RGB or RGBA colour.".format(command, colour)
                            raise_error(ValueError, desc, self.curr_step)
                        
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
                    else:
                        desc = "({0}): Invalid speed value '{1}' (must be [int|float], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                
                case "max_speed":
                    if isinstance(value, (int, float)):
                        if self.units.name in ['IMPERIAL', 'UK']: traci.vehicle.setMaxSpeed(vehicle_id, value / 2.2369362920544)
                        else: traci.vehicle.setMaxSpeed(vehicle_id, value / 3.6)
                    else:
                        desc = "({0}): Invalid max speed value '{1}' (must be [int|float], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                
                case "acceleration":
                    if isinstance(value, (list, tuple)) and len(value) == 2:
                        if not isinstance(value[0], (int, float)):
                            desc = "({0}): Invalid acceleration '{1}' (must be [int|float], not '{2}').".format(command, value[0], type(value[0]).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                        if not isinstance(value[1], (int, float)):
                            desc = "({0}): Invalid laneIndex '{1}' (must be [int|float], not '{2}').".format(command, value[1], type(value[1]).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                        traci.vehicle.setAcceleration(vehicle_id, float(value[0]), float(value[1]))
                    else:
                        desc = "({0}): '{0}' requires 2 parameters (acceleration [int|float], duration [int|float])".format(command)
                        raise_error(TypeError, desc, self.curr_step)

                case "lane_idx":
                    if isinstance(value, (list, tuple)) and len(value) == 2:
                        if not isinstance(value[0], int):
                            desc = "({0}): Invalid laneIndex '{1}' (must be int, not '{2}').".format(command, value[0], type(value[0]).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                        if not isinstance(value[1], (int, float)):
                            desc = "({0}): Invalid duration '{1}' (must be [int|float], not '{2}').".format(command, value[1], type(value[1]).__name__)
                            raise_error(TypeError, desc, self.curr_step)
                        traci.vehicle.changeLane(vehicle_id, value[0], float(value[1]))
                    else:
                        desc = "({0}): '{0}' requires 2 parameters (laneIndex [int], duration [int|float])".format(command)
                        raise_error(TypeError, desc, self.curr_step)

                case "target":
                    if isinstance(value, str):
                        if value not in self._all_edges:
                            desc = "({0}): Edge ID '{1}' not found.".format(command, value)
                            raise_error(KeyError, desc, self.curr_step)
                        traci.vehicle.changeTarget(vehicle_id, value)
                    else:
                        desc = "({0}): Invalid edge ID '{1}' (must be str, not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                
                case "route_id":
                    if isinstance(value, str):
                        if value not in self._all_routes:
                            desc = "({0}): Route ID '{1}' not found.".format(command, value)
                            raise_error(KeyError, desc, self.curr_step)
                        traci.vehicle.setRouteID(vehicle_id, value)
                    else:
                        desc = "({0}): Invalid route ID value '{1}' (must be str, not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                
                case "route_edges":
                    if isinstance(value, (list, tuple)) and all(isinstance(x, str) for x in value):
                        for e_id in value:
                            if e_id not in self._all_edges:
                                desc = "({0}): Edge ID '{1}' in route edges not found.".format(command, e_id)
                                raise_error(KeyError, desc, self.curr_step)
                        traci.vehicle.setRoute(value)
                    else:
                        desc = "({0}): Invalid route egdes value '{1}' (must be (str), not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case _:
                    desc = "Unrecognised command ('{0}').".format(command)
                    raise_error(AttributeError, desc, self.curr_step)

    def get_vehicle_vals(self, vehicle_id: str, data_keys: str|list) -> dict|str|int|float|list:
        """
        Get data values for specific vehicle using a list of data keys.
        :param vehicle_id: Vehicle ID
        :param data_keys: List of keys from [type|length|speed|max_speed|acceleration|position|heading|ts|lane_idx|target|route_id|route_idx|route_edges], or single key
        :return dict: Values by data_key (or single value)
        """

        if not self.vehicle_exists(vehicle_id):
            desc = "Unrecognised vehicle ID given ('{0}').".format(vehicle_id)
            raise_error(KeyError, desc, self.curr_step)

        return_val = False
        if isinstance(data_keys, str):
            data_keys = [data_keys]
            return_val = True
        elif not isinstance(data_keys, (list, tuple)):
            desc = "Invalid data_keys given '{0}' (must be [str|(str)], not '{1}').".format(data_keys, type(data_keys).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
        data_vals = {}
        for data_key in data_keys:
            match data_key:
                case "type":
                    if vehicle_id not in self._known_vehicles.keys():
                        data_vals[data_key] = traci.vehicle.getTypeID(vehicle_id)
                    else: data_vals[data_key] = self._known_vehicles[vehicle_id]["type"]
                case "length":
                    if vehicle_id not in self._known_vehicles.keys():
                        length = traci.vehicle.getLength(vehicle_id)
                        if self.units.name == 'IMPERIAL': length *= 3.28084
                        data_vals[data_key] = length
                    else: data_vals[data_key] = self._known_vehicles[vehicle_id]["length"]
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
                    desc = "Unrecognised key ('{0}').".format(data_key)
                    raise_error(KeyError, desc, self.curr_step)

        if set(data_vals.keys()) != set(data_vals):
            desc = "Invalid data_keys given (must be from accepted list)."
            raise_error(ValueError, desc, self.curr_step)
        if return_val: return list(data_vals.values())[0]
        else: return data_vals
    
    def get_vehicle_data(self, vehicle_id: str, assert_exists: bool = True) -> dict:
        """
        Get data for specified vehicle, updating _known_vehicles dict.
        :param vehicle_id: Vehicle ID
        :param assert_exists: If True, error is raised if vehicle does not exist
        :return dict: Vehicle data dictionary, returns None if does not exist in simulation
        """

        if not self.vehicle_exists(vehicle_id):
            if assert_exists:
                desc = "Unrecognised vehicle ID found ('{0}').".format(vehicle_id)
                raise_error(KeyError, desc, self.curr_step)
            elif not self._suppress_warnings:
                raise_warning("Unrecognised vehicle ID given ('{0}').".format(vehicle_id), self.curr_step)
                return None
        
        vehicle_data = self.get_vehicle_vals(vehicle_id, ("type", "speed", "position", "heading", "ts", "length"))

        if vehicle_id not in self._known_vehicles.keys():
            self._known_vehicles[vehicle_id] = {"type":      vehicle_data["type"],
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
            self._known_vehicles[vehicle_id]["speed"]     = vehicle_data["speed"]
            self._known_vehicles[vehicle_id]["stopped"]   = self.vehicle_is_stopped(vehicle_id, vehicle_data["speed"])
            self._known_vehicles[vehicle_id]["longitude"] = vehicle_data["position"][0]
            self._known_vehicles[vehicle_id]["latitude"]  = vehicle_data["position"][1]
            self._known_vehicles[vehicle_id]["heading"]   = vehicle_data["heading"]
            self._known_vehicles[vehicle_id]["ts"]        = vehicle_data["ts"]
            self._known_vehicles[vehicle_id]["altitude"]  = vehicle_data["position"][2]
            self._known_vehicles[vehicle_id]["last_seen"] = self.curr_step

        vehicle_data = copy(self._known_vehicles[vehicle_id])
        del vehicle_data['last_seen']

        return vehicle_data

    def get_all_vehicle_data(self, types: list = None) -> dict:
        """
        Collects vehicle data from SUMO, by id and/or type (defaults to all vehicles).
        :param types: Type(s) of vehicles to fetch
        :return dict: Stats & vehicle data by id
        """

        all_vehicle_data = {}
        total_vehicle_data = {"no_vehicles": 0, "no_waiting": 0}

        for vehicle_id in self._all_curr_vehicle_ids:

            # Saving known vehicles reduces calls to TraCI by not
            # fetching already known (& unchanging!) data
            if vehicle_id in self._known_vehicles.keys(): vehicle_type = self._known_vehicles[vehicle_id]["type"]
            else: vehicle_type = traci.vehicle.getTypeID(vehicle_id)

            if types is None or (isinstance(types, list) and vehicle_type in types) or (isinstance(types, str) and vehicle_type == types):

                if self._get_individual_vehicle_data:
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
        else:
            desc = "Geometry ID '{0}' not found.".format(geometry_id)
            raise_error(KeyError, desc, self.curr_step)

        return_val = False
        if isinstance(data_keys, str):
            data_keys = [data_keys]
            return_val = True
        elif not isinstance(data_keys, (list, tuple)):
            desc = "Invalid data_keys given '{0}' (must be [str|(str)], not '{1}').".format(data_keys, type(data_keys).__name__)
            raise_error(TypeError, desc, self.curr_step)
        
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
                        desc = "Unrecognised key ('{0}').".format(data_key)
                        raise_error(ValueError, desc, self.curr_step)
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
                        desc = "Unrecognised key ('{0}').".format(data_key)
                        raise_error(ValueError, desc, self.curr_step)
        
        if set(data_vals.keys()) != set(data_vals):
            desc = "Invalid data_keys given (must be from accepted list)."
            raise_error(ValueError, desc, self.curr_step)
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
        
        if geometry_id in self._all_edges:   g_class, g_name = traci.edge, "EDGE"
        elif geometry_id in self._all_lanes: g_class, g_name = traci.lane, "LANE"
        else:
            desc = "Unrecognised egde or lane ID given ('{0}').".format(geometry_id)
            raise_error(KeyError, desc, self.curr_step)
        
        for command, value in kwargs.items():
            match command:
                case "max_speed":
                    if isinstance(value, (int, float)): 
                        if self.units.name in ['IMPERIAL', 'UK']: g_class.setMaxSpeed(geometry_id, value / 2.2369362920544)
                        else: g_class.setMaxSpeed(geometry_id, value / 3.6)
                    else:
                        desc = "({0}): Invalid speed value '{1}' (must be [int|float], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "allowed":
                    if g_name != "LANE":
                        desc = "({0}): Command is only valid for lanes.".format(command)
                        raise_error(ValueError, desc, self.curr_step)
                    if isinstance(value, (list, tuple)):
                        curr_allowed = list(g_class.getAllowed(geometry_id))
                        allowed = tuple(set(curr_allowed + list(value)))
                        g_class.setAllowed(geometry_id, allowed)
                        
                        curr_disallowed = list(g_class.getDisallowed(geometry_id))
                        disallowed = tuple(set(curr_disallowed) - set(value))
                        g_class.setDisallowed(geometry_id, disallowed)
                    else:
                        desc = "({0}): Invalid type list value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "disallowed":
                    if g_name != "LANE":
                        desc = "({0}): Command is only valid for lanes.".format(command)
                        raise_error(ValueError, desc, self.curr_step)
                    if isinstance(value, (list, tuple)):
                        curr_disallowed = list(g_class.getDisallowed(geometry_id))
                        disallowed = tuple(set(curr_disallowed + list(value)))
                        g_class.setDisallowed(geometry_id, disallowed)
                        
                        curr_allowed = list(g_class.getAllowed(geometry_id))
                        allowed = tuple(set(curr_allowed) - set(value))
                        g_class.setAllowed(geometry_id, allowed)
                    else:
                        desc = "({0}): Invalid type list value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case "left_lc":
                    if g_name != "LANE":
                        desc = "({0}): Command is only valid for lanes.".format(command)
                        raise_error(ValueError, desc, self.curr_step)
                    if isinstance(value, (list, tuple)):
                        g_class.setChangePermissions(geometry_id, value[0], 1)
                    else:
                        desc = "({0}): Invalid type list value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)
                
                case "right_lc":
                    if g_name != "LANE":
                        desc = "({0}): Command is only valid for lanes.".format(command)
                        raise_error(ValueError, desc, self.curr_step)
                    if isinstance(value, (list, tuple)):
                        g_class.setChangePermissions(geometry_id, value[0], -1)
                    else:
                        desc = "({0}): Invalid type list value '{1}' (must be [str], not '{2}').".format(command, value, type(value).__name__)
                        raise_error(TypeError, desc, self.curr_step)

                case _:
                    desc = "Unrecognised command ('{0}').".format(command)
                    raise_error(AttributeError, desc, self.curr_step)
                
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
            if v_types != None:
                g_vehicle_ids = [vehicle_id for vehicle_id in g_vehicle_ids if self.get_vehicle_vals(vehicle_id, "type") in v_types]

            if flatten: vehicle_ids += g_vehicle_ids
            else: vehicle_ids[geometry_id] = g_vehicle_ids

        if flatten: vehicle_ids = list(set(vehicle_ids))

        return vehicle_ids
    
    def geometry_exists(self, geometry_id: str|int) -> str|None:
        """
        Get geometry type by ID, if geometry with the ID exists.
        :param geometry_id: Lane or edge ID
        :return str|None:   Geometry type, or None if it does not exist.
        """

        if geometry_id in self._all_edges: return "EDGE"
        elif geometry_id in self._all_lanes: return "LANE"
        else: return None

    def detector_exists(self, detector_id: str) -> str|None:
        """
        Get detector type by ID, if a detector with the ID exists.
        :param detector_id: Detector ID
        :return str|None:   Detector type, or None if it does not exist.
        """

        if detector_id in self.available_detectors.keys():
            return self.available_detectors[detector_id]["type"]
        else: return None

    def route_exists(self, route_id: str) -> str|None:
        """
        Get route edges by ID, if a route with the ID exists.
        :param route_id:  Route ID
        :return str|None: List of route edges, or None if it does not exist.
        """

        if route_id in self._all_routes:
            return traci.route.getEdges(route_id)
        else: return None

    def print_summary(self, save_file=None):
        """
        Prints a summary of a sim_data file or dictionary, listing
        simulation details, vehicle statistics, detectors, controllers,
        tracked edges/junctions and events. The summary can also be saved
        as a '.txt' file.
        :param save_file: '.txt' filename, if given will be used to save summary
        """
        print_summary(self.all_data, save_file)

class TrackedJunction:
    def __init__(self, junc_id: str|int, sim: Simulation, junc_params: dict|str=None) -> None:
        self.id = junc_id
        self.position = traci.junction.getPosition(junc_id)

        self.sim = sim
        self.init_time = sim.curr_step
        self.curr_time = sim.curr_step

        self.track_flow = False
        self.measure_spillback = False
        self.measure_queues = False
        self.is_meter = False
        self.has_tl = junc_id in sim._all_tls

        if self.has_tl:
            state_str = traci.trafficlight.getRedYellowGreenState(junc_id)
            self.m_len = len(state_str)
            self.states, self.curr_state = [], []        
            self.durations = [[] for _ in range(self.m_len)]
            self.avg_green, self.avg_m_red = 0, 0
            self.avg_m_green, self.avg_m_red = [0 for _ in range(self.m_len)], [0 for _ in range(self.m_len)]

        if junc_params != None:
            junc_params = load_params(junc_params, "junc_params", step=self.curr_time)

            if "flow_params" in junc_params.keys():
                flow_params = junc_params["flow_params"]
                if "inflow_detectors" in flow_params.keys() or "outflow_detectors" in flow_params.keys():
                    if not ("inflow_detectors" in flow_params.keys() and "outflow_detectors" in flow_params.keys()):
                        desc = "Both 'inflow_detectors' and 'outflow_detectors' are required parameters to track flow (Junction ID: '{0}').".fomat(self.id)
                        raise_error(KeyError, desc, self.sim.curr_step)
                    else:

                        for detector_id in flow_params["inflow_detectors"]:
                            if detector_id not in self.sim.available_detectors.keys():
                                desc = "Unrecognised detector ID '{0}' given in inflow_detectors (Junction ID: '{1}').".format(detector_id, self.id)
                                raise_error(KeyError, desc, self.sim.curr_step)
                        for detector_id in flow_params["outflow_detectors"]:
                            if detector_id not in self.sim.available_detectors.keys():
                                desc = "Unrecognised detector ID '{0}' given in outflow_detectors (Junction ID: '{1}').".format(detector_id, self.id)
                                raise_error(KeyError, desc, self.sim.curr_step)

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

                self.queue_detector = None
                self.ramp_edges = None

                self.queue_lengths, self.queue_delays = [], []

                if "ramp_edges" in meter_params.keys():
                    self.ramp_edges = meter_params["ramp_edges"]

                    for edge in self.ramp_edges:
                        if edge not in self.sim._all_edges:
                            desc = "Edge ID '{0}' not found.".format(edge)
                            raise_error(KeyError, desc, self.sim.curr_step)

                    self.measure_spillback, self.spillback_vehs = True, []
                    self.measure_queues, self.queue_detector = True, None
                    self.curr_queuing = set([])

                if not self.measure_queues:
                    if "queue_detector" in meter_params.keys():
                        self.measure_queues, self.queue_detector = True, meter_params["queue_detector"]

                        if self.queue_detector not in self.sim.available_detectors.keys():
                            desc = "Unrecognised detector ID given as queue_detector ('{0}').".format(self.queue_detector)
                            raise_error(KeyError, desc, self.sim.curr_step)
                        elif self.sim.available_detectors[self.queue_detector]["type"] != "multientryexit":
                            desc = "Only 'multientryexit' detectors can be used to find queue length (not '{0}').".format(self.sim.available_detectors[self.queue_detector]["type"])
                            raise_error(ValueError, desc, self.sim.curr_step)

                if "init_rate" in meter_params.keys(): self.sim.set_tl_metering_rate(self.id, meter_params["init_rate"])
                else: self.sim.set_tl_metering_rate(self.id, self.max_rate)

            else: self.is_meter = False
    
    def __str__(self): return "<TrackedJunction: '{0}'>".format(self.id)
    def __name__(self): return "TrackedJunction"

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

            else:
                desc = "Cannot update queue length (no detector or entry/exit edges)"
                raise_error(KeyError, desc, self.sim.curr_step)

        if self.measure_spillback:
            spillback_vehs, all_waiting_vehs = 0, 0
            all_loaded_vehicles = self.sim._all_loaded_vehicle_ids
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
    def __name__(self): return "TrackedEdge"

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
            x = self._get_distance_on_road(coors)
            if self.sim.units in ['IMPERIAL']: x *= 0.0006213712
            veh_data.append((x, speed))

        self.step_vehicles.append(veh_data)
            
    def _get_distance_on_road(self, veh_coors):
        line = LineString(self.linestring)
        p = Point(veh_coors)
        p2 = line.interpolate(line.project(p))
        x_val = line.line_locate_point(p2, False)
        x_pct = x_val/line.length
        return x_pct
    
def _get_indent(indent_no, last):
    if indent_no == 0: return ""
    else:
        connector = "└" if last else "├"
        return "  "+"│   "*(indent_no-1) + connector + "── "

def print_dict_tree(dictionary, indent=0):
    for key, val in dictionary.items():
        if isinstance(val, dict):
            print(_get_indent(indent, key == list(dictionary.keys())[-1] and indent==1)+key)
            print_dict_tree(val, indent+1)
        else:
            if isinstance(val, (list, tuple)):
                b1, b2 = ("[", "]") if isinstance(val, list) else ("(", ")")
                type_str = "" if len(val) == 0 else type(val[0]).__name__
                try: shape = np.array(val).shape
                except ValueError: shape = len((val))
                type_str = b1 + type_str + b2 + " " + str(shape)
            else: type_str = type(val).__name__
            print("{0}{1}: {2}".format(_get_indent(indent, key == list(dictionary.keys())[-1] and indent==1), key, type_str))

def print_sim_data_tree(sim_data):
    print_dict_tree({"Simulation Data Structure:": sim_data})

def print_summary(sim_data, save_file=None, tab_width=58):
    """
    Prints a summary of a sim_data file or dictionary, listing
    simulation details, vehicle statistics, detectors, controllers,
    tracked edges/junctions and events.
    :param sim_data: Simulation data dictionary or filepath
    :param save_file: '.txt' filename, if given will be used to save summary
    :param tab_width: Table width
    """
    caller = "{0}()".format(inspect.stack()[0][3])
    if save_file != None:
        if isinstance(save_file, str):
            if not save_file.endswith(".txt"): save_file += ".txt"
            old_stdout = sys.stdout
            sys.stdout = buffer = io.StringIO()
        else:
            desc = "{0}: Invalid save_file type (must be 'str', not '{1}').".format(caller, type(save_file).__name__)
            raise TypeError(desc)
    
    if isinstance(sim_data, str) and sim_data.endswith(".json"):
        if os.path.exists(sim_data):
            with open(sim_data, "r") as fp:
                sim_data = json.load(fp)
        else:
            desc = "{0}: sim_data file '{1}' not found or is invalid.".format(caller, sim_data)
            raise FileNotFoundError(desc)
    elif not isinstance(sim_data, dict):
        desc = "{0}: Invalid sim_data type (must be [str|dict], not '{1}').".format(caller, type(sim_data).__name__)
        raise TypeError(desc)
    elif len(sim_data.keys()) == 0 or "data" not in sim_data.keys():
        desc = "{0}: Invalid sim_data (no data found)."
        raise ValueError(desc)
    
    name = sim_data["scenario_name"]
    if math.floor((tab_width-len(name))/2) != math.ceil((tab_width-len(name))/2):
        tab_width += 1
    
    primary_delineator = " *"+"="*(tab_width+2)+"*"
    secondary_delineator = " *"+"-"*(tab_width+2)+"*"
    tertiary_delineator = " * "+"-"*tab_width+" *"
    
    print(primary_delineator)
    _table_print(sim_data["scenario_name"], tab_width)
    if "scenario_desc" in sim_data.keys():
        desc = sim_data["scenario_desc"]
        print(primary_delineator)
        if tab_width - len("Description: "+desc) > 0:
            _table_print("Description: "+desc, tab_width)
        else:
            _table_print("Description:", tab_width)
            desc_lines = _add_linebreaks(desc, tab_width)
            for line in desc_lines: _table_print(line, tab_width)
    print(primary_delineator)
    
    start_step, end_step = sim_data["start"], sim_data["end"]
    start_time, end_time = datetime.strptime(sim_data["sim_start"], datetime_format), datetime.strptime(sim_data["sim_end"], datetime_format)
    sim_duration, sim_duration_steps = end_time - start_time, end_step - start_step
    if start_time.date() == end_time.date():
        _table_print("Simulation Run: "+start_time.strftime(date_format), tab_width)
        _table_print("{0} - {1} ({2})".format(start_time.strftime(time_format), end_time.strftime(time_format), sim_duration), tab_width)
    else:
        _table_print("Simulation Run: ({0})".format(sim_duration), tab_width)
        _table_print([start_time.strftime(date_format), end_time.strftime(date_format)], tab_width, centre_cols=True)
        _table_print([start_time.strftime(time_format), end_time.strftime(time_format)], tab_width, centre_cols=True)
    
    print(secondary_delineator)
    _table_print(["Number of Steps:", "{0}".format(sim_duration_steps, start_step, end_step)], tab_width)
    _table_print(["Step Length:", sim_data["step_len"]], tab_width)
    _table_print(["Avg. Step Duration:", str(sim_duration.total_seconds() / sim_duration_steps)+"s"], tab_width)
    unit_desc = {"METRIC": "Metric (km, km/h)", "UK": "UK (km, mph)", "IMPERIAL": "Imperial (miles, mph)"}
    _table_print(["Units Type:", unit_desc[sim_data["units"]]], tab_width)
    
    print(primary_delineator)
    _table_print("Data", tab_width)
    print(primary_delineator)
    _table_print("Vehicle Data", tab_width)
    print(secondary_delineator)

    _table_print(["Avg. No. Vehicles:", round(sum(sim_data["data"]["vehicle"]["no_vehicles"])/len(sim_data["data"]["vehicle"]["no_vehicles"]), 2)], tab_width)
    _table_print(["Peak No. Vehicles:", int(max(sim_data["data"]["vehicle"]["no_vehicles"]))], tab_width)
    _table_print(["Final No. Vehicles:", int(sim_data["data"]["vehicle"]["no_vehicles"][-1])], tab_width)
    _table_print(["Individual Data:", "Yes" if "all_vehicles" in sim_data["data"].keys() else "No"], tab_width)
    print(tertiary_delineator)
    _table_print(["Total TTS:", "{0}s".format(round(sum(sim_data["data"]["vehicle"]["tts"]), 2))], tab_width)
    _table_print(["Total Delay:", "{0}s".format(round(sum(sim_data["data"]["vehicle"]["delay"]), 2))], tab_width)
    _table_print(["Individual Data:", "Yes" if "all_vehicles" in sim_data["data"].keys() else "No"], tab_width)

    print(secondary_delineator)
    _table_print("Detectors", tab_width)
    print(secondary_delineator)
    if "detector" not in sim_data["data"].keys() or len(sim_data["data"]["detector"]) == 0:
        _table_print("No detectors found.")
    else:
        ils, mees, unknown, labels = [], [], [], ["Induction Loop Detectors", "Multi-Entry-Exit Detectors", "Unknown Type"]
        for det_id, det_info in sim_data["data"]["detector"].items():
            if det_info["type"] == "inductionloop": ils.append(det_id)
            elif det_info["type"] == "multientryexit": mees.append(det_id)
            else: unknown.append(det_id)
        
        add_spacing = False
        for ids, label in zip([ils, mees, unknown], labels):
            if len(ids) > 0:
                if add_spacing: _table_print(tab_width=tab_width)
                _table_print(label+": ({0})".format(len(ids)), tab_width)
                id_lines = _add_linebreaks(", ".join(ids), tab_width)
                for line in id_lines: _table_print(line, tab_width)
                add_spacing = True
 
    print(secondary_delineator)
    _table_print("Tracked Edges", tab_width)
    print(secondary_delineator)
    if "edges" not in sim_data["data"].keys() or len(sim_data["data"]["edges"]) == 0:
        _table_print("No tracked edges found.")
    else:
        id_lines = _add_linebreaks(", ".join(sim_data["data"]["edges"].keys()), tab_width)
        for line in id_lines: _table_print(line, tab_width)
        
    print(secondary_delineator)
    _table_print("Tracked Junctions", tab_width)
    print(secondary_delineator)
    if "junctions" not in sim_data["data"].keys() or len(sim_data["data"]["junctions"]) == 0:
        _table_print("No tracked junctions found.")
    else:
        for junc_id, junc_info in sim_data["data"]["junctions"].items():
            j_arr = []
            if "tl" in junc_info.keys(): j_arr.append("Signalised")
            if "meter" in junc_info.keys(): j_arr.append("Metered")
            if len(j_arr) == 0: j_arr.append("Uncontrolled")
            _table_print("{0} ({1})".format(junc_id, ", ".join(j_arr)), tab_width)

    if "controllers" in sim_data["data"].keys():
        print(secondary_delineator)
        _table_print("Controllers", tab_width)
        print(secondary_delineator)

        rgs, vsls, unknown, labels = [], [], [], ["Route Guidance", "Variable Speed Limits", "Unknown Type"]
        for cont_id, cont_info in sim_data["data"]["controllers"].items():
            if cont_info["type"] == "RG": rgs.append(cont_id)
            elif cont_info["type"] == "VSL": vsls.append(cont_id)
            else: unknown.append(cont_id)
        
        add_spacing = False
        for ids, label in zip([rgs, vsls, unknown], labels):
            if len(ids) > 0:
                if add_spacing: _table_print(tab_width=tab_width)
                _table_print(label+": ({0})".format(len(ids)), tab_width)
                id_lines = _add_linebreaks(", ".join(ids), tab_width)
                for line in id_lines: _table_print(line, tab_width)
                add_spacing = True
        

    if "events" in sim_data["data"].keys():
        event_statuses = []
        for s in ["scheduled", "active", "completed"]:
            if s in sim_data["data"]["events"].keys() and len(sim_data["data"]["events"][s]) > 0:
                event_statuses.append(s)

        if len(event_statuses) >= 0:
            print(secondary_delineator)
            _table_print("Event IDs & Statuses", tab_width)
            print(secondary_delineator)

            for event_status in event_statuses:
                event_ids = [event["id"] for event in sim_data["data"]["events"][event_status]]
                event_str = "{0}: {1}".format(event_status.title(), ", ".join(event_ids))
                event_lines = _add_linebreaks(event_str, tab_width)
                for line in event_lines: _table_print(line, tab_width)

    print(secondary_delineator)

    if save_file != None:
        sys.stdout = old_stdout
        summary = buffer.getvalue()
        with open(save_file, "w") as fp:
            fp.write(summary)
        print(summary)

def _table_print(strings=None, tab_width=58, side=" | ", padding=" ", centre_cols=False):
    
    if strings == None: strings = ""
    print_str = None
    if isinstance(strings, str):
        print_str = side+_centre_str(strings, tab_width)+side
    elif isinstance(strings, (list, tuple)):
        if centre_cols:
            col_spacing = padding*math.floor((tab_width - sum([len(str(string)) for string in strings])) / (len(strings) + 1))
            print_str = col_spacing+col_spacing.join([str(string) for string in strings])+col_spacing
            print_str = side+_centre_str(print_str, tab_width)+side
        else:
            col_spacing = padding*math.floor((tab_width - sum([len(str(string)) for string in strings])) / (len(strings) - 1))
            print_str = side+col_spacing.join([str(string) for string in strings])+side
    else:
        desc = "_table_print(): Invalid type (must be [str|list|tuple], not '{0}').".format(type(strings).__name__)
        raise TypeError(desc)
    
    if print_str != None:
        print(print_str)
    else: exit()

def _centre_str(string, width, padding=" "):
    if len(string) > width: return string
    else: return padding*math.floor((width-len(string))/2)+string+padding*math.ceil((width-len(string))/2)

def _add_linebreaks(string, width):
    lines = []
    while string != "":
        if " " not in string or len(string) < width:
            lines.append(string)
            break
        elif " " in string and len(string) > width:
            max_len_str = string[:width]
            line_break_index = max_len_str.rfind(" ")
            lines.append(max_len_str[:line_break_index])
            string = string[line_break_index+1:]
    return lines
