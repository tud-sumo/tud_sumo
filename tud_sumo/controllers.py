import traci
from random import random
from utils import *

class VSLController:
    def __init__(self, vsl_id: str|int, vsl_params: dict, simulation):
        from simulation import Simulation

        self.id = vsl_id
        self.c_type = Controller(1)

        self.sim = simulation
        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        self.activated = False

        if "geometry_ids" not in vsl_params.keys():
            desc = "Geometry ID(s) are a required input for VSL controllers (key: 'geometry_ids')."
            raise_error(KeyError, desc, self.sim.curr_step)
        else: self.geometry_ids = vsl_params["geometry_ids"]

        self.geometry = {}
        for geometry_id in self.geometry_ids:
            g_type = self.sim.geometry_exists(geometry_id)
            if g_type in ['EDGE', 'LANE']:
                self.geometry[geometry_id] = {"g_type": g_type, "nc_speed_limit": self.sim.get_geometry_vals(geometry_id, "max_speed"), "avg_speeds": []}
            else:
                desc = "Geometry ID '{0}' not found.".format(geometry_id)
                raise_error(KeyError, desc, self.sim.curr_step)

        self.speed_limits = []
        self.times = []

        self.speed_limit = None if "def_limits" not in vsl_params.keys() else vsl_params["def_limits"]

    def __str__(self): return "<VSLController: '{0}'>".format(self.id)
    def __name__(self): return "VSLController"

    def get_curr_data(self) -> dict:
        vsl_dict = {"type": "VSL", "geometry_data": self.geometry, "init_time": self.init_time, "curr_time": self.curr_time,
                    "speed_limits": self.speed_limits, "times": self.times}
        return vsl_dict
    
    def set_limit(self, speed_limit: int|float|None = None) -> None:
        if self.speed_limit == None and speed_limit == None:
            desc = "Cannot change speed limit as no value given."
            raise_error(ValueError, desc, self.sim.curr_step)
        
        else:
            if speed_limit != None: self.speed_limit = speed_limit

            if self.speed_limit <= 0:
                self.deactivate()
            else:
                self.speed_limits.append(self.speed_limit)
                self.times.append(self.sim.curr_step)
                for geometry_id in self.geometry.keys(): self.sim.set_geometry_vals(geometry_id, max_speed=self.speed_limit)
    
    def deactivate(self) -> None:
        self.activated = False
        self.speed_limit = -1
        for geometry_id in self.geometry.keys():
            self.sim.set_geometry_vals(geometry_id, max_speed=self.geometry[geometry_id]["nc_speed_limit"])
        self.speed_limits.append(self.speed_limit)
        self.times.append(self.sim.curr_step)
    
    def reset(self) -> None:
        self.speed_limits = []
        self.times = []

        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        for g_data in self.geometry.values():
            g_data["avg_speeds"] = []
    
    def update(self) -> None:
        self.curr_time = self.sim.curr_step
        for g_id, g_data in self.geometry.items():
            if self.sim.get_geometry_vals(g_id, "n_vehicles") > 0:
                g_data["avg_speeds"].append(self.sim.get_geometry_vals(g_id, "vehicle_speed"))
            else: g_data["avg_speeds"].append(-1)

class RGController:
    def __init__(self, rg_id: str|int, rg_params: dict, simulation):
        from simulation import Simulation

        self.id = rg_id
        self.c_type = Controller(2)

        self.sim = simulation
        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step

        self.activated = False
        self.activation_times = []
        
        self.old_target = None if "old_destination" not in rg_params.keys() else rg_params["old_destination"]
        self.target = None if "new_destination" not in rg_params.keys() else rg_params["new_destination"]

        self.g_type = None

        self.n_diverted, self.total_vehs = [], []
        self.diverted_vehs = set([])
        self.diversion_pct = 1.0 if "diversion_pct" not in rg_params.keys() else rg_params["diversion_pct"]

        if "vtype" in rg_params.keys():
            self.diversion_vtypes = rg_params["vtype"]
            if not isinstance(self.diversion_vtypes, (list, tuple)):
                self.diversion_vtypes = [self.diversion_vtypes]
        else: self.diversion_vtypes = None

        self.highlight_colour = None if "highlight" not in rg_params.keys() else rg_params["highlight"]
        
        if "detector_ids" not in rg_params.keys():
            desc = "Detector ID(s) are a required input for RG controllers (key: 'detector_ids')."
            raise_error(KeyError, desc, self.sim.curr_step)
        else: self.detector_ids = rg_params["detector_ids"]
        
        if not isinstance(self.detector_ids, (list, tuple)): 
            self.detector_ids = [self.detector_ids]

        self.detector_info = {}
        for detector_id in self.detector_ids:
            if detector_id not in self.sim.available_detectors.keys():
                desc = "Detector ID '{0}' not found.".format(detector_id)
                raise_error(KeyError, desc, self.sim.curr_step)
            elif self.sim.available_detectors[detector_id]["type"] != 'inductionloop':
                desc = "Invalid RG detector type (must be 'inductionloop' not '{0}')".format(self.sim.available_detectors[detector_id]["type"])
                raise_error(KeyError, desc, self.sim.curr_step)
            else: self.detector_info[detector_id] = {"location": traci.inductionloop.getLaneID(detector_id)}

    def __str__(self): return "<RGController: '{0}'>".format(self.id)
    def __name__(self): return "RGController"

    def get_curr_data(self) -> dict:
        rg_dict = {"type": "RG", "detector_ids": self.detector_info, "init_time": self.init_time, "curr_time": self.curr_time}

        if self.old_target != None: rg_dict["init_target"] = self.old_target
        
        if self.g_type == 'EDGE': rg_dict["new_target_edge"] = self.target
        elif self.g_type == 'ROUTE': rg_dict["new_route"] = self.target

        rg_dict["activation_times"] = self.activation_times
        rg_dict["n_diverted"] = self.n_diverted
        rg_dict["diversion_pct"] = self.diversion_pct
        
        return rg_dict
    
    def activate(self, old_target: str|int|None = None, new_target: str|int|None = None, diversion_pct: float|None = None, highlight_colour: str|None = None) -> dict:
        if self.target == None and new_target == None:
            desc = "Cannot activate as no target given."
            raise_error(ValueError, desc, self.sim.curr_step)
        else:
            if not self.activated:
                if new_target != None: self.target = new_target

                if self.sim.geometry_exists(self.target) == 'EDGE': self.g_type = 'EDGE'
                elif self.sim.route_exists(self.target) != None: self.g_type = 'ROUTE'
                else:
                    desc = "Route or edge ID '{1}' not found".format(self.target)
                    raise_error(KeyError, desc, self.sim.curr_step)

                self.activated = True
                self.activation_times.append(self.curr_time)
                if diversion_pct != None: self.diversion_pct = diversion_pct
                if old_target != None: self.old_target = old_target

                if highlight_colour != None: self.highlight_colour = highlight_colour
            elif not self.sim.suppress_warnings: raise_warning("RG controller '{0}' is already activated.".format(self.id), self.sim.curr_step)

    def deactivate(self) -> None:
        
        if not self.activated:
            if not self.sim.suppress_warnings: raise_warning("Controller already deactivated.", self.sim.curr_step)
        else:
            self.activated = False

            if len(self.activation_times) > 0:
                activation_time = self.activation_times[-1]
                self.activation_times[-1] = (activation_time, self.curr_time)
            
            else: self.activation_times = [(self.init_time, self.curr_time)]
    
    def reset(self) -> None:
        self.n_diverted, self.total_vehs = [], []
        self.diverted_vehs = set([])
        self.activation_times = []

        self.init_time = self.sim.curr_step
        self.curr_time = self.sim.curr_step
    
    def update(self) -> None:
        """
        Performs route guidance on vehicles passing over the detector.
        """

        self.curr_time = self.sim.curr_step
        n_diverted, total_vehs = 0, 0

        if self.activated:
            all_vehs = self.sim.get_last_step_detector_vehicles(self.detector_ids, v_types=self.diversion_vtypes, flatten=True)
            for veh_id in all_vehs:
                total_vehs += 1
                if veh_id not in self.diverted_vehs and random() <= self.diversion_pct:
                    if self.old_target == None or self.sim.get_vehicle_vals("new_destination") == self.old_target:
                        if self.g_type == 'EDGE':
                            self.sim.set_vehicle_vals(veh_id, destination=self.target, highlight=self.highlight_colour)
                        elif self.g_type == 'ROUTE':
                            self.sim.set_vehicle_vals(veh_id, route_id=self.target, highlight=self.highlight_colour)

                        n_diverted += 1
                        self.diverted_vehs.add(veh_id)
        
        self.total_vehs.append(total_vehs)
        self.n_diverted.append(n_diverted)