import json, os, math, inspect, numpy as np
from enum import Enum
from datetime import datetime
import traci.constants as tc

date_format = "%d/%m/%Y"
time_format = "%H:%M:%S"
datetime_format = "%d/%m/%Y, %H:%M:%S"

unit_desc = {"METRIC": "Metric (km, km/h)", "UK": "UK (km, mph)", "IMPERIAL": "Imperial (miles, mph)"}
time_desc = {"s": "Seconds", "m": "Minutes", "hr": "Hours"}

traci_constants = {"vehicle": {
                                "speed": tc.VAR_SPEED, "is_stopped": tc.VAR_SPEED, "max_speed": tc.VAR_MAXSPEED, "acceleration": tc.VAR_ACCELERATION,
                                "position": tc.VAR_POSITION, "altitude": tc.VAR_POSITION3D, "heading": tc.VAR_ANGLE, "lane_idx": tc.VAR_LANE_INDEX,
                                "destination": tc.VAR_ROUTE, "route_id": tc.VAR_ROUTE_ID, "route_idx": tc.VAR_ROUTE_INDEX, "route_edges": tc.VAR_ROUTE},
                   "detector": {
                                "vehicle_count": tc.LAST_STEP_VEHICLE_NUMBER, "vehicle_ids": tc.LAST_STEP_VEHICLE_ID_LIST, "speed": tc.LAST_STEP_MEAN_SPEED,
                                "halting_no": tc.LAST_STEP_VEHICLE_HALTING_NUMBER, "occupancy": tc.LAST_STEP_OCCUPANCY, "last_detection": tc.LAST_STEP_TIME_SINCE_DETECTION},
                   "geometry": {
                                "vehicle_count": tc.LAST_STEP_VEHICLE_NUMBER, "vehicle_ids": tc.LAST_STEP_VEHICLE_ID_LIST, "speed": tc.LAST_STEP_MEAN_SPEED,
                                "halting_no": tc.LAST_STEP_VEHICLE_HALTING_NUMBER, "occupancy": tc.LAST_STEP_OCCUPANCY}
                  }

class Units(Enum):
    METRIC = 1
    IMPERIAL = 2
    UK = 3

class Controller(Enum):
    VSL = 1
    RG = 2
    METER = 3

def raise_error(error, desc, curr_sim_step=None):
    caller = "{0}.{1}()".format(inspect.currentframe().f_back.f_locals['self'].__name__(), inspect.currentframe().f_back.f_code.co_name)
    error_msg = "{0}: {1}".format(caller, desc)
    if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
    raise error(error_msg)
    try:
        caller = "{0}.{1}()".format(inspect.currentframe().f_back.f_locals['self'].__name__(), inspect.currentframe().f_back.f_code.co_name)
        error_msg = "{0}: {1}".format(caller, desc)
        if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
        raise error(error_msg)
    except Exception:
        print("tud_sumo: An unknown error occurred.")
        raise KeyError("no")

def raise_warning(desc, curr_sim_step=None):
    caller = "{0}.{1}()".format(inspect.currentframe().f_back.f_locals['self'].__name__(), inspect.currentframe().f_back.f_code.co_name)
    warning_msg = "(WARNING) {0}: {1}".format(caller, desc)
    if curr_sim_step != None: warning_msg = "(step {0}) ".format(curr_sim_step) + warning_msg
    print(warning_msg)

def convert_time_units(time_vals, unit, step_len):
    if not isinstance(time_vals, list): time_vals = [time_vals]
    if unit == "s" and step_len != None:
        time_vals = [val * step_len for val in time_vals]
    elif unit == "m" and step_len != None:
        time_vals = [(val * step_len)/60 for val in time_vals]
    elif unit == "hr" and step_len != None:
        time_vals = [(val * step_len)/3600 for val in time_vals]

    if len(time_vals) == 1: return time_vals[0]
    else: return time_vals

def get_time_steps(data_vals, unit, step_len=None, start=0):
    time_vals = list(range(len(data_vals)))
    time_vals = [val + start for val in time_vals]

    return convert_time_units(time_vals, unit, step_len)
         
def get_time_str():
    date_str = datetime.now().strftime(datetime_format)
    return date_str

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

def load_params(parameters: str|dict, params_name: str|None = None, step: int|None = None) -> dict:
    """
    Load parameters file. Handles either dict or json file.
    :param parameters: Parameters dict or filepath
    :param params_name: Parameter dict function (for error messages)
    :param step: Current simulation step (for error messages)
    :return dict: Parameters dict
    """

    caller = "{0}.{1}()".format(inspect.currentframe().f_back.f_locals['self'].__name__(), inspect.currentframe().f_back.f_code.co_name)
    if step != None: caller = "(step {0}) {1}".format(step, caller)
    if params_name == None: params_name = "parameter input"

    if not isinstance(parameters, (dict, str)):
        raise TypeError("{0}: Invalid {1} (must be [dict|filepath (str)], not '{2}').".format(caller, params_name, type(parameters).__name__))
    elif isinstance(parameters, str) and parameters.endswith(".json"):
        if os.path.exists(parameters):
            with open(parameters, "r") as fp:
                parameters = json.load(fp)
        else: raise FileNotFoundError("{0}: Parameters file '{1}' not found.".format(caller, parameters))
    elif isinstance(parameters, str): raise ValueError("{0}: Invalid parameters file '{1}' (must be '.json' file).".format(caller, parameters))

    return parameters

def get_aggregated_data(data_vals, time_steps, interval):

    agg_start, agg_data, agg_steps = 0, [], []
    while agg_start < len(data_vals):
        period_data = data_vals[agg_start:int(min(agg_start+interval, len(data_vals)))]
        period_data = [max(val, 0) for val in period_data]
        agg_data.append(sum(period_data) / len(period_data))
        period_data = time_steps[agg_start:int(min(agg_start+interval, len(time_steps)))]
        agg_steps.append(period_data[-1])
        agg_start += interval

    return agg_data, agg_steps

def get_axis_lim(data_vals, end_buff = 0.05):
    """
    Get axis limit rounded to nearest 1000/100/10 (with buffer).
    :param data_vals: Single (max) axis value, or list of values
    :param end_buff: Minimum axis buffer above maximum value (default to 5%)
    :return float: Axis limit
    """
    
    pct_buff = 1 + end_buff
    if isinstance(data_vals, (list, tuple)): max_val = max(data_vals)
    else:
        max_val = data_vals
        data_vals = [data_vals]

    if max_val == min(data_vals) == 0: return 1

    for scale in [1000, 100, 10, 1]:
        if max_val >= scale:
            return math.ceil((max_val * pct_buff) / (scale / 5)) * (scale / 5)
        
    return max_val * pct_buff

def limit_vals_by_range(time_steps, data_vals=None, time_range=None):
    """
    For plotting, to limit data values between to those within a given time range.
    :param time_steps: List of time step values
    :param data_vals:  List of data values, same length as time_steps, (if not given, only time_steps is limited)
    :param time_range: (1x2) array containing minimum and maximum time step values (if not given, data is returned unchanged)
    :return (list):    Limited time_steps (& data_vals if given, where tuple returned is (steps, vals))
    """

    if time_range == None or (time_range[0] < time_steps[0] and time_range[1] > time_steps[-1]):
        if data_vals != None: return time_steps, data_vals
        else: return time_steps

    if data_vals != None:
        new_vals, new_steps = [], []
        for val, step in zip(data_vals, time_steps):
            if step >= time_range[0] and step <= time_range[1]:
                new_vals.append(val)
                new_steps.append(step)
            elif step > time_range[1]:
                return new_steps, new_vals
    else: return [step for step in time_steps if step >= time_range[0] and step <= time_range[1]]

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