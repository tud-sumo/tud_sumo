import json, os, math, inspect
from enum import Enum
from datetime import datetime
import traci.constants as tc
import pickle as pkl
from difflib import SequenceMatcher

date_format = "%d/%m/%Y"
time_format = "%H:%M:%S"
datetime_format = "%d/%m/%Y, %H:%M:%S"

unit_desc = {"METRIC": "Metric (km, km/h)", "UK": "UK (km, mph)", "IMPERIAL": "Imperial (miles, mph)"}
time_desc = {"s": "Seconds", "m": "Minutes", "hr": "Hours"}

traci_constants = {"vehicle": {
                                "speed": tc.VAR_SPEED, "is_stopped": tc.VAR_SPEED, "max_speed": tc.VAR_MAXSPEED, "acceleration": tc.VAR_ACCELERATION,
                                "position": tc.VAR_POSITION, "altitude": tc.VAR_POSITION3D, "heading": tc.VAR_ANGLE, "edge_id": tc.VAR_ROAD_ID,
                                "lane_idx": tc.VAR_LANE_INDEX, "route_id": tc.VAR_ROUTE_ID, "route_idx": tc.VAR_ROUTE_INDEX},
                   "detector": {
                                "vehicle_count": tc.LAST_STEP_VEHICLE_NUMBER, "vehicle_ids": tc.LAST_STEP_VEHICLE_ID_LIST, "lsm_speed": tc.LAST_STEP_MEAN_SPEED,
                                "halting_no": tc.LAST_STEP_VEHICLE_HALTING_NUMBER, "lsm_occupancy": tc.LAST_STEP_OCCUPANCY, "last_detection": tc.LAST_STEP_TIME_SINCE_DETECTION},
                   "geometry": {
                                "vehicle_count": tc.LAST_STEP_VEHICLE_NUMBER, "vehicle_ids": tc.LAST_STEP_VEHICLE_ID_LIST, "vehicle_speed": tc.LAST_STEP_MEAN_SPEED,
                                "halting_no": tc.LAST_STEP_VEHICLE_HALTING_NUMBER, "vehicle_occupancy": tc.LAST_STEP_OCCUPANCY}
                  }

valid_detector_val_keys = ["type", "position", "vehicle_count", "vehicle_ids", "lsm_speed", "halting_no", "last_detection", "lsm_occupancy", "avg_vehicle_length"]

valid_set_vehicle_val_keys = ["colour", "highlight", "speed", "max_speed", "acceleration", "lane_idx", "destination", "route_id", "route_edges", "speed_safety_checks", "lc_safety_checks"]
valid_get_vehicle_val_keys = ["type", "length", "departure", "origin", "destination", "route_edges"] + list(traci_constants["vehicle"].keys())

valid_set_geometry_val_keys = ["max_speed", "allowed", "disallowed", "left_lc", "right_lc"]
valid_get_geometry_val_keys = ["avg_vehicle_length", "curr_travel_time", "ff_travel_time", "emissions", "length", "connected_edges", "incoming_edges", "outgoing_edges", "junction_ids",
                               "street_name", "n_lanes", "lane_ids", "max_speed", "egde_id", "n_links"] + valid_set_geometry_val_keys + list(traci_constants["geometry"].keys())

class Units(Enum):
    METRIC = 1
    IMPERIAL = 2
    UK = 3

class Controller(Enum):
    VSL = 1
    RG = 2
    METER = 3

def _get_type_str(valid_types, connector=","):

    if isinstance(valid_types, (list, tuple)):
        types = []

        for vt in valid_types:
            if isinstance(vt, type): types.append(vt.__name__)
            elif isinstance(vt, tuple):
                vt_arr = []
                for t in vt:
                    if isinstance(t, type): vt_arr.append(t.__name__)
                    else: vt_arr.append(type(t).__name__)
                types.append("[{0}]".format("|".join(vt_arr)))

        return "[{0}]".format(connector.join(types))
    
    elif isinstance(valid_types, type): return valid_types.__name__

    else: return valid_types

def validate_list_types(values, valid_types, element_wise=False, param_name=None, curr_sim_step=None, return_validity=False):

    invalid, desc = False, None
    param_name = param_name if param_name != None else "input list"

    inv_idx, inv_val = None, None

    if isinstance(values, (list, tuple)) and isinstance(valid_types, tuple):
        if element_wise:
            if len(values) == len(valid_types):
                for idx, (value, valid_type) in enumerate(zip(values, valid_types)):
                    if not isinstance(value, valid_type):
                        invalid, inv_idx, inv_val = True, idx, value
                        valid_types = valid_type
                        break
            else:
                desc = "Invalid {0} (must have length '{1}').".format(param_name, len(valid_types))
                invalid = True

        else:
            for idx, value in enumerate(values):
                if type(value) not in valid_types:
                    invalid, inv_idx, inv_val = True, idx, value
                    break

    if isinstance(values, (list, tuple)):
        for idx, value in enumerate(values):
            if not isinstance(value, valid_types):
                invalid, inv_idx, inv_val = True, idx, value
                break

    else:
        invalid = True
        if not return_validity:
            if desc == None: desc = "Invalid {0} (must be '[list|tuple]', not '{1}').".format(param_name, type(values).__name__)
            if 'self' in inspect.currentframe().f_back.f_locals:
                caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
            else: caller = ""
            caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
            error_msg = "{0}: {1}".format(caller, desc)
            if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
            raise TypeError(error_msg)
    
    if invalid and not return_validity:
        if desc == None: desc = "Invalid value found in {0} at idx {1} (must be '{3}', '{2}' is '{4}').".format(param_name, inv_idx, inv_val, _get_type_str(valid_types, "|"), type(value).__name__)
        if 'self' in inspect.currentframe().f_back.f_locals:
            caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
        else: caller = ""
        caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
        error_msg = "{0}: {1}".format(caller, desc)
        if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
        raise TypeError(error_msg)
    
    if return_validity: return invalid
    else: return values

def validate_type(value, valid_types, param_name=None, curr_sim_step=None, return_validity=False):

    if not isinstance(value, valid_types) or (isinstance(value, tuple) and type(value) not in valid_types):
        if return_validity: return False
        else:
            param_name = param_name if param_name != None else "input"
            val_str = "" if len(str(value)) >= 20 else "'{0}' ".format(value)
            desc = "Invalid {0} {1}(must be '{2}', not '{3}').".format(param_name, val_str, _get_type_str(valid_types, "|"), type(value).__name__)
            if 'self' in inspect.currentframe().f_back.f_locals:
                caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
            else: caller = ""
            caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
            error_msg = "{0}: {1}".format(caller, desc)
            if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
            raise TypeError(error_msg)
    
    if return_validity: return True
    else: return value

def raise_error(error, desc, curr_sim_step=None):
    if 'self' in inspect.currentframe().f_back.f_locals:
        caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
    else: caller = ""
    caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
    error_msg = "{0}: {1}".format(caller, desc)
    if curr_sim_step != None: error_msg = "(step {0}) ".format(curr_sim_step) + error_msg
    raise error(error_msg)

def raise_warning(desc, curr_sim_step=None):
    if 'self' in inspect.currentframe().f_back.f_locals:
        caller = inspect.currentframe().f_back.f_locals['self'].__name__() + "."
    else: caller = ""
    caller += "{0}()".format(inspect.currentframe().f_back.f_code.co_name)
    warning_msg = "(WARNING) {0}: {1}".format(caller, desc)
    if curr_sim_step != None: warning_msg = "\n(step {0}) ".format(curr_sim_step) + warning_msg
    print(warning_msg)

def convert_units(values, orig_units, new_units, step_length=1, keep_arr=False):

    if isinstance(values, (int, float)): values = [values]
    elif not isinstance(values, (list, tuple)):
        desc = "Invalid values '{0}' type (must be [int|float|list|tuple], not '{1}').".format(values, type(values).__name__)
        raise_error(TypeError, desc)
    elif False in [isinstance(val, (int, float)) for val in values]:
        desc = "Invalid values '{0}' type (list must be only contain [int|float]).".format(values)
        raise_error(ValueError, desc)

    c_mats = {"distance":
                  {"units": ["metres", "kilometres", "yards", "feet", "miles"],
                   "matrix": [[1, 1/1000, 1.093613, 3.280840, 1/1609.344498], # m
                              [1000, 1, 1093.613298, 3280.839895, 1/1.609344], # km
                              [1/1.093613,  1/1093.613298, 1, 3, 1/1760], # yd
                              [1/3.280840,  1/3280.839895, 1/3, 1, 1/5280], # ft
                              [1609.344498, 1.609344, 1760, 5280, 1]] # mi
                  },
              "speed":
                  {"units": ["m/s", "kmph", "mph"],
                   "matrix": [[1, 3.6, 2.236936], # m/s
                              [1/3.6, 1, 1/1.609344], # kmph
                              [1/2.236936, 1.609344, 1]] # mph
                  },
              "time":
                  {"units": ["steps", "seconds", "minutes", "hours"],
                   "matrix": [[1, step_length, step_length/60, step_length/3600], # steps
                              [1/step_length, 1, 1/60, 1/3600], # s
                              [60/step_length, 60, 1, 1/60], # m
                              [3600/step_length, 3600, 60, 1]] # hr
                  }
             }
    
    o_class = [key for key, val in c_mats.items() if orig_units in val["units"]]
    n_class = [key for key, val in c_mats.items() if new_units in val["units"]]

    if len(o_class) == 0:
        desc = "Unknown unit '{0}'.".format(orig_units)
        raise_error(ValueError, desc)
    else: o_class = o_class[0]

    if len(n_class) == 0:
        desc = "Unknown unit '{0}'.".format(new_units)
        raise_error(ValueError, desc)
    else: n_class = n_class[0]

    if o_class != n_class:
        desc = "Invalid conversion (cannot convert '{0}' ({1}) -> '{2}' ({3}))".format(o_class, orig_units, n_class, new_units)
        raise_error(ValueError, desc)
    
    o_idx = c_mats[o_class]["units"].index(orig_units)
    n_idx = c_mats[o_class]["units"].index(new_units)
    values = [value * c_mats[o_class]["matrix"][o_idx][n_idx] for value in values]
    if len(values) == 1 and not keep_arr: values = values[0]

    return values

def get_time_steps(data_vals, unit, step_len=None, start=0):
    time_vals = list(range(len(data_vals)))
    time_vals = [val + start for val in time_vals]

    return convert_units(time_vals, "steps", unit, step_len)
         
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
    
    Args:
        `filepath` (str): SUMO filename
    
    Returns:
        str: Scenario name
    """
    cfg_file = filepath.split('/')[-1]
    if cfg_file.endswith('.sumocfg'): cfg_file = cfg_file.removesuffix('.sumocfg')
    elif cfg_file.endswith('.neteditcfg'): cfg_file = cfg_file.removesuffix('.neteditcfg')
    elif cfg_file.endswith('.add.xml'): cfg_file = cfg_file.removesuffix('.add.xml')
    elif cfg_file.endswith('.rou.xml'): cfg_file = cfg_file.removesuffix('.rou.xml')
    elif cfg_file.endswith('.net.xml'): cfg_file = cfg_file.removesuffix('.net.xml')
    return cfg_file

def load_params(parameters: str|dict, params_name: str|None = None, step: int|None = None) -> dict:
    """
    Load parameters file. Handles either dict or json file.
    
    Args:
        `parameters` (str, dict): Parameters dict or filepath
        `params_name` (str, None): Parameter dict function (for error messages)
        `step` (int, None): Current simulation step (for error messages)
    
    Returns:
        dict: Parameters dict
    """

    caller = "{0}.{1}()".format(inspect.currentframe().f_back.f_locals['self'].__name__(), inspect.currentframe().f_back.f_code.co_name)
    if step != None: caller = "(step {0}) {1}".format(step, caller)
    if params_name == None: params_name = "parameter input"

    if not isinstance(parameters, (dict, str)):
        raise TypeError("{0}: Invalid {1} (must be [dict|filepath (str)], not '{2}').".format(caller, params_name, type(parameters).__name__))
    elif isinstance(parameters, str):
        if parameters.endswith(".json"): r_class, r_mode = json, "r"
        elif parameters.endswith(".pkl"): r_class, r_mode = pkl, "rb"    
        else:
            raise ValueError("{0}: Invalid parameters file '{1}' (must be '.json' or '.pkl' file).".format(caller, parameters))

        if os.path.exists(parameters):
            with open(parameters, r_mode) as fp:
                parameters = r_class.load(fp)
        else: raise FileNotFoundError("{0}: Parameters file '{1}' not found.".format(caller, parameters))

    return parameters

def get_aggregated_data(data_vals, time_steps, interval, avg=True):
    """
    Args:
        `data_vals` (list): Data values array
        `time_steps` (list): List of time step values
        `interval` (int): Aggregation interval
        `avg` (bool): Denotes whether to average/sum values

    Returns:
        (list, list): Aggregated data, time steps
    """
    
    agg_start, agg_data, agg_steps = 0, [], []
    while agg_start < len(data_vals):
        period_data = data_vals[agg_start:int(min(agg_start+interval, len(data_vals)))]
        period_data = [max(val, 0) for val in period_data]
        if avg: agg_data.append(sum(period_data) / len(period_data))
        else: agg_data.append(sum(period_data))
        period_data = time_steps[agg_start:int(min(agg_start+interval, len(time_steps)))]
        agg_steps.append(period_data[-1])
        agg_start += interval

    return agg_data, agg_steps

def get_axis_lim(data_vals, end_buff = 0.05):
    """
    Get axis limit rounded to nearest 1000/100/10 (with buffer).
    
    Args:
        `data_vals`: Single (max) axis value, or list of values
        `end_buff`: Minimum axis buffer above maximum value (default to 5%)
    
    Returns:
        float: Axis limit
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

def limit_vals_by_range(time_steps, data_vals=None, time_range=None) -> list|tuple:
    """
    For plotting, to limit data values between to those within a given time range.
    
    Args:
        `time_steps`: List of time step values
        `data_vals`: List of data values, same length as time_steps, (if not given, only time_steps is limited)
        `time_range`: (1x2) array containing minimum and maximum time step values (if not given, data is returned unchanged)
    
    Returns:
        list: Limited time_steps (& data_vals if given, where tuple returned is (steps, vals))
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

def get_most_similar_string(input_string, valid_strings, req_similarity=0.6):
    best_match, best_similarity = None, 0
    for valid_string in valid_strings:
        similarity = SequenceMatcher(None, input_string, valid_string).ratio()
        if similarity > best_similarity and similarity >= req_similarity:
            best_match, best_similarity = valid_string, similarity
    return best_match

def test_valid_string(input_string, valid_strings, param_name, case_sensitive=True, req_similarity=0.6):
    if not isinstance(input_string, str):
        desc = "Invalid {0} '{1}' (must be 'str', not '{2}').".format(param_name, input_string, type(input_string).__name__)
        return TypeError, desc

    if case_sensitive: found = input_string in valid_strings
    elif not case_sensitive: found = input_string.upper() in [string.upper() for string in valid_strings]
    else: found = False

    if not found:
        valid_strings.sort()
        desc = "Unknown {0} '{1}'".format(param_name, input_string)
        closest = get_most_similar_string(input_string, valid_strings, req_similarity)
        if closest != None: desc = desc + ". Did you mean '{0}'?".format(closest)
        else: desc = desc + " (must be ['{0}']).".format("'|'".join(valid_strings))

        return ValueError, desc

    return None, None

def test_input_dict(input_dict, valid_params, dict_name="", required=None) -> str:

    desc = None
    dict_name = dict_name + " " if dict_name != "" else dict_name

    if len(input_dict) == 0:
        desc = "Empty {0}parameters given.".format(dict_name)
        return (ValueError, desc)
    
    if isinstance(required, bool) and required: required = list(valid_params.keys())
    if required != None:
        missing_keys = list(set(required) - set(input_dict.keys()))
        if len(missing_keys) > 0:
            desc = "Missing required {0}parameters ('{1}').".format(dict_name, "', '".join(list(missing_keys)))
            return (KeyError, desc)
        
    for key, item in input_dict.items():
        if key not in valid_params:
            desc = "Unrecognised {0}parameter '{1}'".format(dict_name, key)
            close_match = get_most_similar_string(key, valid_params.keys())
            desc = "{0}. Did you mean '{1}'?".format(desc, close_match) if close_match != None else desc + "."
            return (KeyError, desc)
        if not isinstance(item, valid_params[key]):
            if isinstance(valid_params[key], (list, tuple)):
                type_str = "[{0}]".format("|".join([str(val.__name__) for val in valid_params[key]]))
            else: type_str == valid_params[key].__name__
            desc = "Invalid {0} type '{1}' (must be '{2}' not '{3}')".format(key, item, type_str, type(item).__name__)
            return (TypeError, desc)
        
    return (None, None)