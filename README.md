# TU Delft SUMO Wrapper

Python SUMO wrapper, using traci, written for the DIAMoND Lab at TU Delft by Callum Evans. The full documentation & examples can be found at [tud-sumo.github.io/docs/](https://tud-sumo.github.io/docs/).

![Header - Delft](https://i.imgur.com/dYrHOPY.png)

## Requirements 

Python 3.10 or later is required. Dependencies are; `tqdm`, `matplotlib`, `mpl-tools`, `shapely`, `sumolib` and `traci`.

## Latest Updates

### Demand Generation, More Getters and General Fixes 

#### Added
  - Added dynamic demand generation with `Simulation.load_demand()` and `Simulation.add_demand()` functions.
  - Added demand to `sim_data` dictionary.
  - Added number of waiting vehicles to collected data.
  - Added more getters for data: `Simulation.get_[no_vehicles/no_waiting/tts/delay]()`.
  - Added `Simulation.get_[junction/tracked_junction/tracked_edge/event/controller]_exists()` functions.
  - Added `Simulation.get_[junction/tracked_junction/tracked_edge/event/controller]_ids()` functions.
  - Added `Simulation.remove_controllers()` function.
  - Added `Plotter.plot_od_demand()` function.
  - Added `utils.conver_units()` function and removed `utils.convert_time_units()`.
  - Added `EventScheduler.get_event_ids()` function to get status of event.
  - Added basic `Plotter.plot_fundamental_diagram()` function.
  - Added `'incoming_edges'`, `'outgoing_edges'`, `'junction_ids'`, `'ff_travel_time'` and `'curr_travel_time'` to `Simulation.get_geometry_vals()`.

#### Changes
  - Improved error handling.
  - Added vehicle type filter to `Simulation.get_all_vehicle_data()` function.
  - Changed `Simulation.vehicle_departed()` to `Simulation.vehicle_to_depart()`.
  - Changed `Simulation.tracked_juncs` to `Simulation.tracked_junctions`.
  - Changed 's' to 'seconds', 'm' to 'minutes', 'hr' to 'hours' wherever they appear.
  - `VSLController` data now stored in the same format as `RGController` with an activation times list.
  - Simplified activation times data in `RGController`.
  - Changed `VSLController.set_limit()` to `VSLController.set_speed_limit()`.
  - Changed `'EDGE'` and `'LANE'` to lowercase when getting geometry types.
  - Changed `'stopped'` to `'is_stopped'` in vehicle data.
  - Removed `'event_n_steps'` and `'event_duration'` from event parameters.

## Contact Information

TUD-SUMO is developed by Callum Evans in the DIAMoND lab of TU Delft. For any questions, feedback or bug reports, please contact Callum Evans or submit a query using the form [here](https://forms.office.com/e/pMnGaheier).