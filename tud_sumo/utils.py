import json, os, math, numpy as np
from enum import Enum

class Units(Enum):
    METRIC = 1
    IMPERIAL = 2
    UK = 3

class Controller(Enum):
    VSL = 1
    RG = 2
    METER = 3

def get_cumulative_arr(arr: list, start: int=0) -> list:
    arr = [0] + arr
    for i in range(start + 1, len(arr)):
        arr[i] += arr[i - 1]
    return arr[1:]

def get_scenario_name(filepath: str) -> str:
    """
    Get scenario name from filepath.
    :param filepath: '.sumocfg' or '.neteditcfg' filename
    :return str: Scenario name
    """
    cfg_file = filepath.split('/')[-1]
    if cfg_file.endswith('.sumocfg'): cfg_file = cfg_file.removesuffix('.sumocfg')
    elif cfg_file.endswith('.neteditcfg'): cfg_file = cfg_file.removesuffix('.neteditcfg')
    return cfg_file

def load_params(parameters: str|dict, caller: str, step: int, params_name: str) -> dict:
    """
    Load parameters file. Handles either dict or json file.
    :param parameters: Parameters dict or filepath
    :param caller: Function calling load_params() (for error messages)
    :param step: Current simulation step (for error messages)
    :param params_name: Parameter dict function (for error messages)
    :return dict: Parameters dict
    """
    
    if not isinstance(parameters, (dict, str)):
        raise TypeError("(step {0}) {1} [utils.load_params()]: Invalid {2} (must be [dict|filepath (str)], not '{3}').".format(step, caller, params_name, type(parameters).__name__))
    elif isinstance(parameters, str) and parameters.endswith(".json"):
        if os.path.exists(parameters):
            with open(parameters, "r") as fp:
                parameters = json.load(fp)
        else: raise FileNotFoundError("(step {0}) {1} [utils.load_params()]: Parameters file '{2}' not found.".format(step, caller, parameters))
    elif isinstance(parameters, str): raise ValueError("(step {0}) {1} [utils.load_params()]: Invalid parameters file '{2}' (must be '.json' file).".format(step, caller, parameters))

    return parameters

def get_axis_lim(data_vals, end_buff = 0.05):
    """
    Get axis limit rounded to nearest 1000/100/10 (with buffer).
    :param data_vals: Single (max) axis value, or list of values
    :param end_buff: Minimum axis buffer above maximum value (default to 5%)
    :return float: Axis limit
    """

    pct_buff = 1 + end_buff
    if isinstance(data_vals, (list, tuple)): max_val = max(data_vals)
    else: max_val = data_vals

    for scale in [1000, 100, 10, 1]:
        if max_val >= scale:
            return math.ceil((max_val * pct_buff) / (scale / 5)) * (scale / 5)
        
    return max_val * pct_buff

def get_space_time_matrix(sim_data, edge_ids=None, x_resolution=1, y_resolution=10, upstream_at_top=True):
    """
    Converts edge vehicle positions and speeds to a space-time matrix.
    :param sim_data: Simulation data (dict)
    :param edge_ids: List of tracked edge IDs or single ID
    :param x_resolution: X axis matrix resolution
    :param y_resolution: Y axis matrix resolution
    :param upstream_at_top: If true, upstream values are displayed at the top of the matrix
    :return np.array: NumPy matrix
    """

    if "edges" in sim_data["data"].keys():
        edge_data = sim_data["data"]["edges"]
        if edge_ids == None: edge_ids = list(edge_data.keys())
        elif not isinstance(edge_ids, (list, tuple)): edge_ids = [edge_ids]
        for edge_id in edge_ids:
            if edge_id in edge_data.keys():
                n_steps = len(edge_data[edge_id]["step_vehicles"])
            else: raise KeyError("Plotter.plot_space_time_diagram(): Edge '{0}' not found in tracked edge.".format(edge_id))
    else: raise KeyError("Plotter.plot_space_time_diagram(): No edges tracked during the simulation.")

    total_len = sum([edge_data[e_id]["length"] for e_id in edge_ids])
    
    n_vehicle_matrix = np.zeros((math.ceil(total_len / y_resolution), math.ceil(n_steps / x_resolution)))
    speed_matrix = np.zeros((math.ceil(total_len / y_resolution), math.ceil(n_steps / x_resolution)))

    edge_offset = 0
    for e_id in edge_ids:
        e_data = sim_data["data"]["edges"][e_id]

        step_vehicles = e_data["step_vehicles"]
        
        edge_length = e_data["length"]
        
        for idx, step_data in enumerate(step_vehicles):
            for veh_data in step_data:
                x_val = math.floor(idx / x_resolution)
                y_val = math.floor(((veh_data[0] * edge_length) + edge_offset) / y_resolution)

                total_speed = (speed_matrix[y_val][x_val] * n_vehicle_matrix[y_val][x_val]) + veh_data[1]
                n_vehicle_matrix[y_val][x_val] += 1
                speed_matrix[y_val][x_val] = total_speed / n_vehicle_matrix[y_val][x_val]

        edge_offset += edge_length

    if not upstream_at_top:
        n_vehicle_matrix = np.flip(n_vehicle_matrix, 0)
        speed_matrix = np.flip(speed_matrix, 0)

    return speed_matrix, n_vehicle_matrix