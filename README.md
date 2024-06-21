# TU Delft SUMO Wrapper

Python SUMO wrapper, using traci, written for the AIM Lab at TU Delft by Callum Evans. Usage and output examples can be found in the `tud_sumo/examples` directory.

![Header - Delft](https://i.imgur.com/dYrHOPY.png)

## Requirements 

Python 3.10 or later is required. Dependencies are; `tqdm`, `matplotlib`, `shapely`, `sumolib` and `traci`.

## Latest Updates

### Pickle support, dynamic incidents & validation

#### New Additions
 - Added pickle support and `.pkl` version of example data. All files can be saved/read as either pickle or JSON files.
 - Added `Simulation.cause_incident()` to (randomly) create an incident in the simulation.
 - Added `highlight` to `Simulation.set_vehicle_vals()`.
 - Added `Scheduler.get_event_status()`
 - Added `remove_affected_vehicles`, `speed_safety_checks` and `lc_safety_checks` options to events.
 - Added a toggle for `speed_safety_checks` and `lc_safety_checks` to `Simulation.set_vehicle_vals()`.
 - Added `connected_edges` to `Simulation.get_geometry_vals()` to return incoming and outgoing edges for a given edge.
 - Added `Simulation.load_objects()` function to load edge/junction/phase/controller/event parameters in a single dict or file.
 - Added `type`, `position` to `Simulation.get_detector_vals()`.
 - Added `edge_id` as a vehicle subscription.
 - Added `Simulation.get_[vehicle|geometry|detector]_ids()` and `Simulation.get_vehicle_types()` functions.

#### Improvements
 - Improved validation for dictionary inputs (checking valid/required keys and data types).
 - All getter and setter 'vals' functions (`Simulation.[get|set]_[vehicle|geometry|detector]_vals()`) now accept multiple IDs. Getters will return a dictionary containing each object's information, whilst setters set values for all objects.
 - Improved type checking & resulting errors.
 - Improved seed checking, now allowing strings/integers/"random".
 - Trip data shows `"removal"` time instead of `"departure"` time if a vehicle is removed manually.

#### Changes
 - Changed all instances of `vtype`/`v_type` to `vehicle_type`.
 - Changed `highlight` to `colour` in `Simulation.set_vehicle_vals()`.
 - Changed detector `"occupancy"` and `"speed"` to `"lsm_occupancy"` and `"lsm_speed"` (last mean speed) in data files.
 - Added `Simulation._vehicles_in()` and `Simulation._vehicles_out()` functions.
 - `Simulation.get_last_step_detector_data()` is changed to `get_detector_vals` for consistency.
 - Changed `print_sim_data()` function to `print_sim_data_struct()`
 - Lane index is included in step vehicle data for tracked edges.
 - Removed `get_space_time_matrix()`

#### Fixes
 - Fixed duplicate IDs when adding objects.
 - Fixed strange behaviour with vehicle removal.

## Contact Information

For any questions, feedback or bug reports, please contact Callum Evans at [c.evans@tudelft.nl](mailto:c.evans@tudelft.nl).