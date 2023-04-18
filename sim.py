
import os, sys, traci
from functools import reduce
from tqdm import tqdm
from random import randint
import matplotlib.pyplot as plt
import math

class Simulation:
    def __init__(self, main_junction: str = None, verbose: bool = False) -> None:
        path_tools = os.path.join(os.environ.get("SUMO_HOME"), 'tools')

        if path_tools in sys.path: pass
        else: sys.path.append(path_tools)

        self.gui = False
        self.verbose = verbose or '-v' in sys.argv

        self.def_junc = main_junction

        self.detector_list = []
        self.step_length = 0.5

        self.known_vehicles = {}
        self.light_changes = 0

    def start(self, config_file: str | None = None, net_file: str | None = None, route_file: str | None = None, add_file: str | None = None, cmd_options: list | None = None, warnings: bool = False, gui: bool = False) -> None:
        """
        Intialises SUMO simulation.
        :param config_file: Location of '.sumocfg' file (can be given instead of net_file)
        :param net_file:    Location of '.net.xml' file (can be given instead of config_file)
        :param route_file:  Location of '.rou.xml' route file
        :param add_file:    Location of '.add.xml' additional file
        :param cmd_options: List of any other command line options
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
        
        sumoCMD += ["--step-length", str(self.step_length)]
        if not warnings or not self.verbose: sumoCMD.append("--no-warnings")

        traci.start(sumoCMD)
        
        self.update_lights()
        self.curr_step = 0
        self.time_val = traci.simulation.getTime()
        self.detector_list = list(traci.multientryexit.getIDList())
    
    def end(self):
        traci.close()

    def step_to(self, end_step: int | None = None, n_steps: int | None = None, n_light_changes: int | None = None, detector_list: list | None = None, prev_data: dict | None = None, vTypes: list | str | None = None, cumulative_data: bool = False) -> dict:
        """
        Step through simulation from the current time until end_step, aggregating
        data during this period.
        :param end_step:        End point for stepping through simulation
        :param n_steps:         Perform n steps of the simulation (given instead of end_steps)
        :param n_light_changes: Run for n light changes (across all junctions)
        :param detector_list:   List of detector IDs to collect data from (defaults to all)
        :param prev_data:       If given, new data is appended to this dictionary
        :param vTypes:          Vehicle type(s) to collect data of (list of types or string, defaults to all)
        :param cumulative_data: Denotes whether to get cumulative veh count and TTS values
        :return dict:           All data collected through the time period, separated by detector
        """

        if detector_list == None: detector_list = self.detector_list
        start_time = self.curr_step

        if end_step == None and n_steps != None: end_step = self.curr_step + n_steps
        elif end_step == None and n_light_changes != None: end_step, init_changes = math.inf, self.light_changes
        elif end_step == None: raise ValueError("No time value given for step_to() function.")

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
                    all_data["data"]["detector"][detector_id] = {"speeds": [], "veh_counts": []}
                all_data["data"]["vehicle"] = {"no_vehicles": [], "tts": [], "delay": []}

            for detector_id in last_step_data["detector"].keys():
                if detector_id not in all_data["data"]["detector"].keys(): raise KeyError("Unrecognised detector ID found: "+detector_id)
                for data_key, data_val in last_step_data["detector"][detector_id].items():
                    all_data["data"]["detector"][detector_id][data_key].append(data_val)

            for data_key, data_val in last_step_data["vehicle"].items():
                all_data["data"]["vehicle"][data_key].append(data_val)

            if traci.simulation.getMinExpectedNumber() == 0:
                traci.close()
                print('Ended simulation early: no vehicles remaining.')
                break

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

    def step(self, detector_list: list | None = None, vTypes: list | str | None = None) -> dict:
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

        if detector_list == None: detector_list = self.detector_list
        for detector_id in detector_list:
            speed = traci.multientryexit.getLastStepMeanSpeed(detector_id)
            veh_count = traci.multientryexit.getLastStepVehicleNumber(detector_id)
            data["detector"][detector_id] = {"speeds": speed, "veh_counts": veh_count}

        total_v_data, all_v_data = self.get_vehicle_data(types=vTypes)
        data["vehicle"]["no_vehicles"] = total_v_data["no_vehicles"]
        data["vehicle"]["tts"] = total_v_data["no_vehicles"] * self.step_length
        data["vehicle"]["delay"] = total_v_data["no_waiting"] * self.step_length

        self.curr_step += 1

        return data, all_v_data
    
    def set_phases(self, junc_phases: dict, start_phase: int = 0) -> None:
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

    def set_phase(self, junction: str, phase_no: int) -> None:
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

    def set_lights(self, junction_id: str, light_str: str) -> None:
        """
        Set lights to a specific setting. Will be overwritten at next
        light update.
        :param junction_id: Junction ID
        :param light_str: SUMO light string
        """
        traci.trafficlight.setRedYellowGreenState(junction_id, light_str)

    def update_lights(self, junction_ids: list | str | None = None) -> None:
        """
        Update light settings for given junctions
        :param junction_ids: Junction ID, or list of IDs (defaults to all)
        """

        if junction_ids is None: junction_ids = self.junc_phases.keys()
        elif isinstance(junction_ids, str): junction_ids = [junction_ids]

        for junction_id in junction_ids:
            curr_setting = traci.trafficlight.getRedYellowGreenState(junction_id)
            self.light_changes += 1
            new_phase = self.junc_phases[junction_id]["phases"][self.junc_phases[junction_id]["curr_phase"]]
            if '-' in new_phase:
                new_phase = new_phase.split()
                new_phase = "".join([new_phase[i] if new_phase[i] != '-' else curr_setting[i] for i in range(len(new_phase))])
            traci.trafficlight.setRedYellowGreenState(junction_id, new_phase)

    def get_vehicle_data(self, ids: list | None = None, types: list | str | None = None, get_individual_data: bool = True) -> dict:
        """
        Collects vehicle data from SUMO, by id and/or type (defaults to all vehicles)
        :param ids: IDs of vehicles to fetch
        :param types: Type(s) of vehicles to fetch
        :param get_individual_data: Denotes whether to get individual speed, location & heading data
        :return dict: Vehicle data by id
        """
        if ids == None: ids = list(traci.vehicle.getIDList())

        all_vehicle_data = {}
        total_vehicle_data = {"no_vehicles": 0, "no_waiting": 0}

        for vehicle_id in ids:

            # Saving known vehicles reduces calls to traci by not
            # fetching already known data
            if vehicle_id in self.known_vehicles.keys(): vehicle_type = self.known_vehicles[vehicle_id]["type"]
            else:
                vehicle_type = traci.vehicle.getVehicleClass(vehicle_id)
                self.known_vehicles[vehicle_id] = {"type": vehicle_type}

            if types is None or (isinstance(types, list) and vehicle_type in types) or (isinstance(types, str) and vehicle_type == types):
                speed = traci.vehicle.getSpeed(vehicle_id)
                lon, lat, alt = traci.vehicle.getPosition3D(vehicle_id)

                if get_individual_data:

                    if vehicle_id in self.known_vehicles.keys() and "length" in self.known_vehicles[vehicle_id].keys():
                        length = self.known_vehicles[vehicle_id]["length"]
                    else:
                        length = traci.vehicle.getLength(vehicle_id)
                        self.known_vehicles[vehicle_id]["length"] = length
                
                    all_vehicle_data[vehicle_id] = {"type": vehicle_type,
                                                "longitude": lon,
                                                "latitude": lat,
                                                "speed": speed,
                                                "length": length,
                                                "heading": traci.vehicle.getAngle(vehicle_id),
                                                "ts": traci.vehicle.getDeparture(vehicle_id),
                                                "altitude": alt
                                            }
                    
                total_vehicle_data["no_vehicles"] += 1
                if speed < 0.1: total_vehicle_data["no_waiting"] += 1

        return total_vehicle_data, all_vehicle_data
    
def get_cumulative_arr(arr: list, start: int) -> list:
    for i in range(start, len(arr)):
        arr[i] += arr[i - 1]
    return arr

def plot_vehicle_data(all_data: dict, dataset: str | list, save_fig: None | str = None) -> None:
    """
    Plots all collected vehicle or detector data.
    :param all_data: Data collected from the Simulation.step_to() function
    :param dataset: Dataset key (either "vehicle" or ["detector", detector_id])
    """

    default_labels = {"no_vehicles": "No. of Vehicles", "tts": "TTS (s)", "delay": "Delay (s)",
                      "speeds": "Avg. Speed (km/h)", "veh_counts": "No. of Vehicles"}

    if isinstance(dataset, str):
        if dataset in all_data["data"].keys(): data = all_data["data"][dataset]
        else: raise KeyError("Dataset not found: '{0}'".format(dataset))
        title = dataset.title()
    elif isinstance(dataset, list):
        data = all_data["data"]
        for key in dataset:
            if key in data.keys(): data = data[key]
            else: raise KeyError("Dataset not found: '{0}'".format(dataset[-1]))
        title = ': '.join(dataset)

    else: raise ValueError("Invalid dataset key type, must be [int|str], not '{0}'".format(type(dataset).__name__))

    fig, axes = plt.subplots(len(data))
    start, end, step = all_data["start"], all_data["end"], all_data["step_len"]

    for idx, (ax, (data_key, data_vals)) in enumerate(zip(axes, data.items())):
        ax.plot([x * step for x in range(start, end)], data_vals, zorder=3)
        ax.set_title(data_key)
        if data_key in default_labels.keys(): ax.set_ylabel(default_labels[data_key])
        if idx < len(axes) - 1: ax.tick_params('x', labelbottom=False)
        else: ax.set_xlabel('Time (s)')
        ax.set_xlim([start * step, (end - 1) * step])
        ax.set_ylim([0, max(data_vals) * 1.05])
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

    fig.suptitle(title)
    fig.tight_layout()

    if save_fig is None: plt.show(block=True)
    else: plt.savefig(save_fig)

def get_phases(Qcolor: dict, Qtime: dict, M_map: dict, blocks_lim: int | None = None, simulation_time: int | None = None) -> dict:
    """
    Converts Qcolor, Qtime dictionaries into light settings and times
    for SUMO.
    :param Qcolor: Qcolor movement dictionary
    :param Qtime:  Qtime movement timings dictionary
    :param M_map:  Movement mapping dictionary
    :param blocks_lim: Limit on the number of blocks to convert (defaults to all)
    :param simulation_time: Current simulation time, if not given, new phases start immediately
    :return dict: Phases dictionary
    """
    
    if set(Qcolor.keys()) != set(Qtime.keys()) != set(M_map.keys()):
        raise ValueError("Movement IDs do not match.")

    movements = Qcolor.keys()
    if "0" in str(reduce(lambda m1, m2: '{0:b}'.format(int(m1, 2) ^ int(m2, 2)), list(M_map.values()))):
        raise ValueError("Invalid 'M_map' value given.")
    else:
        mapping = {m: [bool(int(val)) for val in values] for m, values in M_map.items()}

    num_lanes = [len(M_map[m]) for m in movements][0]
    
    if len([m for m in movements if len(Qcolor[m]) != len(Qtime[m])]) != 0:
        raise ValueError("A movement in Qtime and Qcolor does not have matching lengths.")

    min_phase_start = min([times[0] for times in Qtime.values()])
    if len(set([times[0] for times in Qtime.values()])) != 1:
        for m in movements:
            m_start_time = Qtime[m][0]
            if m_start_time != min_phase_start:
                Qcolor[m].insert(0, '-')
                Qtime[m].insert(0, min_phase_start)

    if isinstance(simulation_time, int):
        if simulation_time != min_phase_start:
            for m in movements:
                Qcolor[m].insert(0, '-')
                Qtime[m].insert(0, simulation_time)

    all_times = []
    for m in movements: all_times += Qtime[m]
    all_times = sorted(list(set(all_times)))
    times = [all_times[i] - all_times[i - 1] for i in range(1, len(all_times))]

    if blocks_lim == None or blocks_lim > len(times): blocks_lim = len(times)
    curr_time = min([Qtime[m][0] for m in movements])
    curr_block = {m: 0 for m in movements}

    phases_dict = {"phases": [], "times": []}
    light_arr = ["-"]*num_lanes
    
    for time in times[:blocks_lim]:
        for m in movements:
            block = curr_block[m]
            if block < len(Qcolor):
                light_arr = [Qcolor[m][block][0] if l else light_arr[idx] for idx, l in enumerate(mapping[m])]

            if block < len(Qtime) and curr_time + time == Qtime[m][block + 1]: # Weird behaviour when not all movements have the same amount of phases / end at different times?
                curr_block[m] += 1                                             # _map = {0: "100", 1: "010", 2: "001"}, _qcolor = {0: ['r', 'y', 'g'], 1: ['y', 'g', 'r'], 2: ['g', 'r', 'g']}, _qtime = {0: [100, 105, 110], 1: [100, 103, 110], 2: [100, 105, 108]}

        light_string = "".join(light_arr)
        phases_dict["phases"].append(light_string)
        curr_time += time

    phases_dict["times"] = times[:blocks_lim]

    return phases_dict

def generate_rnd_mapping(n_movements: int, map_length: int) -> dict:
    mapping = {m: ["0"]*map_length for m in range(n_movements)}
    for i in range(map_length): mapping[randint(0, n_movements-1)][i] = "1"
    mapping = {m: "".join(arr) for m, arr in mapping.items()}
    return mapping

if __name__ == "__main__":
    default_junction = "J2"

    default_mapping = {0: "11111111000000000000000000000000",
                       1: "00000000111111100000000000000000",
                       2: "00000000000000011111110000000000",
                       3: "00000000000000000000001111111111"}

                                            # '-' means the light stays the same
    default_Qcolor = {0: ['g',  'y',  'ra', '-', 'g',  'y',  'ra', 'rb', 'g',  '-'],
                      1: ['g',  'y',  'ra', '-', 'g',  'y',  'ra', 'rb', 'g',  '-'],
                      2: ['ra', 'rb', 'g',  'y', 'ra', 'rb', 'g',  'y',  'ra', '-'],
                      3: ['ra', 'rb', 'g',  'y', 'ra', 'rb', 'g',  'y',  'ra', '-']}
    
    # NOTE: Here, it never does the last phase in Qcolor, as it doesn't have a time for it to end.
    #       This because the Simulation class uses phase duration, instead of phase start times as in Qtime.
    #       It also assumes cycles should loop (which can be stopped by having the last phase of infinite length),
    #       but these assumptions may not be an issue if you are generating new phase dictionaires every few steps

    default_Qtime = {0: [0, 6.224149003633258, 9.224149003633258, 26.499999993318994, 29.999999993318994, 42.999999993318994, 45.999999993318994, 59.499999993318994, 62.999999993318994, 68],
                     1: [0, 6.224149003633258, 9.000000000000000, 19.224149003633258, 26.499999993318994, 39.499999993318994, 42.499999993318994, 59.499999993318994, 62.999999993318994, 68],
                     2: [0, 6.224149003633258, 9.724149003633258, 26.499999993318994, 29.499999993318994, 42.999999993318994, 46.499999993318994, 59.499999993318994, 62.499999993318994, 68],
                     3: [0, 6.224149003633258, 9.724149003633258, 15.724149003633258, 18.724149003633258, 42.999999993318994, 46.499999993318994, 59.499999993318994, 62.499999993318994, 68]} 

    # get_phases(default_Qcolor, default_Qtime, default_mapping) returns:
    # {'phases': ['gggggggggggggggrrrrrrrrrrrrrrrrr', 'yyyyyyyyyyyyyyyrrrrrrrrrrrrrrrrr', ... ], -> Phase light settings
    #  'times':  [6.224149003633258, 2.775850996366742, 0.2241490036332578 ... ]}                -> Phase durations

    sim = Simulation(main_junction=default_junction)

    sim.set_phases({default_junction: get_phases(default_Qcolor, default_Qtime, default_mapping)})
    sim.start("files/pompidou.sumocfg", gui=False)

    sim_data = sim.step_to(end_step=3600)                    # End siulation at step 3600
    sim_data = sim.step_to(n_steps=3600, prev_data=sim_data) # Run for 3600 steps (use prev_data to append)
    # Example sim_data in example_data.json

    sim.end()

    # Plot vehicle measures (n_vehicles, tts, delay)
    plot_vehicle_data(sim_data, dataset="vehicle")

    # Plot data from detector 'cars_n' (n_vehicles, speeds)
    plot_vehicle_data(sim_data, dataset=["detector", "cars_n"])