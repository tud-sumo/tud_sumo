import json, math, os
from random import random

class EventScheduler:
    def __init__(self, sim):
        from simulation import Simulation

        self.sim = sim
        self.scheduled_events = {}
        self.active_events = {}
        self.completed_events = {}

    def __dict__(self):
        schedule_dict = {}

        if len(self.scheduled_events) > 0:
            schedule_dict["scheduled"] = []
            for event in self.scheduled_events.values(): schedule_dict["scheduled"].append(event.__dict__())

        if len(self.active_events) > 0:
            schedule_dict["active"] = []
            for event in self.active_events.values(): schedule_dict["active"].append(event.__dict__())

        if len(self.completed_events) > 0:
            schedule_dict["completed"] = []
            for event in self.completed_events.values(): schedule_dict["completed"].append(event.__dict__())

        return schedule_dict
    
    def add_events(self, events):

        if isinstance(events, Event): self.scheduled_events[events.id] = events

        else:
            if isinstance(events, dict):
                for event_id, event_params in events.items():
                    if isinstance(event_params, Event): self.scheduled_events[event_id] = event_params
                    elif isinstance(event_params, dict): self.scheduled_events[event_id] = Event(event_id, event_params, self.sim)
                    else: raise TypeError("(step {0}) EventScheduler.init: Invalid event_params dict (must contain [dict|Event], not '{1}').".format(self.sim.curr_step, type(event_params).__name__))

            elif isinstance(events, str):
                if events.endswith(".json"):
                    if os.path.exists(events):
                        with open(events, "r") as fp:
                            events = json.load(fp)
                        for event_id, event_params in events.items():
                            self.scheduled_events[event_id] = Event(event_id, event_params, self.sim)
                    else: raise FileNotFoundError("(step {0}) EventScheduler.init: Event parameters file '{1}' not found.".format(self.sim.curr_step, events))
                else: raise ValueError("(step {0}) EventScheduler.init: Event junction parameters file '{1}' (must be '.json' file).".format(self.sim.curr_step, events))
                
            elif hasattr(events, "__iter__") and all([isinstance(event, Event) for event in events]):
                for event in events: self.scheduled_events[event.id] = event
            
            else:
                raise TypeError("(step {0}) EventScheduler.init: Invalid event_params (must be [dict|filepath|(Event)], not '{1}').".format(self.sim.curr_step, type(events).__name__))

    def update_events(self):
        
        scheduled_event_ids = list(self.scheduled_events.keys())
        for event_id in scheduled_event_ids:
            event = self.scheduled_events[event_id]
            if event.start_time <= self.sim.curr_step:
                event.start()
                del self.scheduled_events[event_id]
                self.active_events[event_id] = event

        active_events = list(self.active_events.keys())
        for event_id in active_events:
            event = self.active_events[event_id]
            if event.is_active():
                event.run()
            else:
                del self.active_events[event_id]
                self.completed_events[event_id] = event

class Event:
    def __init__(self, event_id, event_params, sim):
        self.id = event_id
        self.sim = sim

        if not isinstance(event_params, dict) and not isinstance(event_params, str):
            raise TypeError("(step {0}) Event.init: Invalid event_params (must be [dict|filepath (str)], not '{1}').".format(self.sim.curr_step, type(event_params).__name__))
        elif isinstance(event_params, str):
            if event_params.endswith(".json"):
                if os.path.exists(event_params):
                    with open(event_params, "r") as fp:
                        event_params = json.load(fp)
                else: raise FileNotFoundError("(step {0}) Event.init: Event parameters file '{1}' not found.".format(self.sim.curr_step, event_params))
            else: raise ValueError("(step {0}) Event.init: Event junction parameters file '{1}' (must be '.json' file).".format(self.sim.curr_step, event_params))

        if "start_time" in event_params.keys(): self.start_time = event_params["start_time"]
        else: raise KeyError("(step {0}) Event.init: Event 'start_time' is not given and is required.".format(self.curr_step))

        self.end_time = math.inf if "end_time" not in event_params.keys() else event_params["end_time"]

        if "end_time" in event_params.keys():
            self.end_time = event_params["end_time"]
        elif "event_duration" in event_params.keys():
            self.end_time = self.start_time + event_params["event_duration"]
        else:
            raise KeyError("(step {0}) Event.init: Event 'end_time' and 'event_duration is not given and 1 is required.".format(self.curr_step))

        if "edge" not in event_params.keys() and "vehicle" not in event_params.keys():
            raise KeyError("(step {0}) Event.init: Neither 'edge' or 'vehicle' parameters are given and one or both is required.".format(self.curr_step))
        
        if "edge" in event_params.keys():
            if "actions" not in event_params["edge"].keys() and "egde_ids" not in event_params["edge"].keys():
                raise KeyError("(step {0}) Event.init: Edge events require 'actions' and 'edge_ids' parameters.".format(self.curr_step))
            
            self.edge_ids = event_params["edge"]["edge_ids"]
            self.e_actions, self.e_base = event_params["edge"]["actions"], {}

        else: self.e_actions = None

        if "vehicle" in event_params.keys():
            if "actions" not in event_params["vehicle"].keys() and "locations" not in event_params["vehicle"].keys():
                raise KeyError("(step {0}) Event.init: Vehicle events require 'actions' and 'locations' parameters.".format(self.curr_step))
            
            self.locations = event_params["vehicle"]["locations"]
            
            self.v_effect_dur = math.inf if "effect_dur" not in event_params["vehicle"].keys() else event_params["vehicle"]["effect_dur"]
            self.v_actions, self.v_base, self.affected_vehicles = event_params["vehicle"]["actions"], {}, {}

            if "vehicle_types" in event_params["vehicle"].keys():
                self.v_types = event_params["vehicle"]["vehicle_types"]
            else: self.v_types = None

            self.v_prob = 1 if "effect_probability" not in event_params["vehicle"].keys() else event_params["vehicle"]["effect_probability"]
            
            if "vehicle_limit" in event_params["vehicle"].keys():
                self.vehicle_limit, self.total_affected_vehicles = event_params["vehicle"]["vehicle_limit"], 0
            else:
                self.vehicle_limit, self.total_affected_vehicles = math.inf, 0

            self.highlight = None if "highlight" not in event_params["vehicle"].keys() else event_params["vehicle"]["highlight"]

            for data_key in ["acceleration", "lane_idx"]:
                if data_key in self.v_actions.keys():
                    self.v_actions[data_key] = (self.v_actions[data_key], self.v_effect_dur)

        else: self.v_actions = None

    def __dict__(self):

        event_dict = {"id": self.id, "start_time": self.start_time, "end_time": self.end_time}
        if self.e_actions != None:
            event_dict["edge"] = {"edge_ids": self.edge_ids, "actions": self.e_actions}
        if self.v_actions != None:
            event_dict["vehicle"] = {"locations": self.locations, "actions": self.v_actions,
                                     "effect_duration": self.v_effect_dur, "n_affected": self.total_affected_vehicles}
            
        return event_dict

    def start(self):

        if self.e_actions != None:
            for edge_id in self.edge_ids:
                self.e_base[edge_id] = self.sim.get_geometry_vals(edge_id, list(self.e_actions.keys()))
                self.sim.set_geometry_vals(edge_id, **self.e_actions)
                self.e_effects_active = True

                if "allowed" in self.e_base[edge_id].keys():
                    self.e_base[edge_id]["disallowed"] = self.e_actions["allowed"]
                    del self.e_base[edge_id]["allowed"]
                if "disallowed" in self.e_base[edge_id].keys():
                    self.e_base[edge_id]["allowed"] = self.e_actions["disallowed"]
                    del self.e_base[edge_id]["disallowed"]

        self.e_effects_active = self.e_actions != None
        self.v_effects_active = self.v_actions != None
        

    def run(self):

        if self.e_actions != None:
            if self.end_time <= self.sim.curr_step and self.e_effects_active:
                for edge_id in self.edge_ids:
                    self.sim.set_geometry_vals(edge_id, **self.e_base[edge_id])
                self.e_effects_active = False

        if self.v_actions != None:
            if self.end_time > self.sim.curr_step:
                new_vehicles = []
                for location in self.locations:
                    if self.sim.geometry_exists(location) != None:
                        new_vehicles += self.sim.get_last_step_geometry_vehicles(location, v_types = self.v_types, flatten = True)
                    elif self.sim.detector_exists(location) != None:
                        new_vehicles += self.sim.get_last_step_detector_vehicles(location, v_types = self.v_types, flatten = True)
                    else: raise KeyError("(step {0}) EventScheduler.update_events: Object '{0}' has unknown type (must be detector or geometry ID).".format(self.sim.curr_step, location))
                new_vehicles = list(set(new_vehicles))
                new_vehicles = [vehicle_id for vehicle_id in new_vehicles if vehicle_id not in self.affected_vehicles.keys()]

                for vehicle_id in new_vehicles:
                    if self.total_affected_vehicles < self.vehicle_limit and random() < self.v_prob:
                        self.v_base[vehicle_id] = self.sim.get_vehicle_vals(vehicle_id, list(self.v_actions.keys()))
                        if "speed" in self.v_base[vehicle_id].keys(): self.v_base[vehicle_id]["speed"] = -1
                        self.sim.set_vehicle_vals(vehicle_id, **self.v_actions)

                        # If self.v_effect_dur is a number, this is used as effect duration, else if it is "EVENT",
                        # all vehicle effects will be stopped once the event is over.
                        if isinstance(self.v_effect_dur, str) and self.v_effect_dur.upper() == "EVENT":
                            self.affected_vehicles[vehicle_id] = {"start_effect": self.sim.curr_step, "end_effect": self.end_time}
                        else:
                            self.affected_vehicles[vehicle_id] = {"start_effect": self.sim.curr_step, "end_effect": self.sim.curr_step + self.v_effect_dur}

                        if self.highlight != None:
                            self.sim.set_vehicle_vals(vehicle_id, highlight=self.highlight)
                        self.total_affected_vehicles += 1

            affected_vehicles_ids = list(self.affected_vehicles.keys())
            for vehicle_id in affected_vehicles_ids:
                if not self.sim.vehicle_exists(vehicle_id):
                    del self.affected_vehicles[vehicle_id]
                    continue
                
                elif self.affected_vehicles[vehicle_id]["end_effect"] <= self.sim.curr_step:
                    
                    if "acceleration" in self.v_base[vehicle_id].keys():
                        del self.v_base[vehicle_id]["acceleration"]
                    if "lane_idx" in self.v_base[vehicle_id].keys():
                        del self.v_base[vehicle_id]["lane_idx"]
                    if self.highlight != None:
                        self.v_base[vehicle_id]["highlight"] = None

                    self.sim.set_vehicle_vals(vehicle_id, **self.v_base[vehicle_id])
                    del self.affected_vehicles[vehicle_id]

                    if len(self.affected_vehicles.keys()) == 0 and self.end_time <= self.sim.curr_step:
                        self.v_effects_active = False

    def is_active(self):
        return self.e_effects_active or self.v_effects_active