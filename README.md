# TU Delft SUMO Wrapper

Python SUMO wrapper, using traci, written for AIM Lab at TU Delft. Download with [pip](https://pypi.org/project/tud-sumo/) or from [GitHub](https://github.com/calluume/tud_sumo).

## Requirements 

Required packages are: `tqdm`, `matplotlib` and `traci`.

## Updates

This is version '1.2.1', with the changelog:
  - `sim.step_to()` is changed to `sim.step_through()` to better reflect usage
  - Added `sim.vehicle_exists()` function
  - `sim.all_curr_vehicle_ids` list
  - Fixed heading and ts not updating in `sim.known_vehicles`
  - Improved documentation

Check changes and previous versions through the project's [GitHub repository](https://github.com/calluume/tud_sumo).

## Usage examples

```python    
from tud_sumo import tud_sumo

sim = tud_sumo.Simulation()
sim.start("files/scenario.sumocfg", gui=True)
```

The phase dictionary is a dictionary containing the phases and times. `math.inf` will set a light permanently, and `'-'` will keep that movement on the same setting as the previous phase. `sim.set_phases()` will then start these settings within the simulation on the next time step. If no phase dictionary is given, the simulation will use the default SUMO logic.

```python
phase_dictionary = {'junctionID': {'phases': [ 'ggrryy', 'rryygg', 'yyggrr', 'rrrr--' ], # Phase light settings
                                   'times':  [ 10.12345, 10.12345, 10.12345, math.inf ]}} # Phase durations

sim.set_phases(phase_dictionary)
```

Use `sim.step_through()` to run through the simulation, collecting and aggregating the simulation data. This will return a dictionary containing all data collected, which can be used again as `prev_data` if continuing the simulation, meaning all data is appended and aggregated correctly throughout the whole runtime. This will contain all detector and vehicle information, although detectors and vehicle types can be specified using `detector_list` and `vTypes` respectively. To improve performance, set `sim.get_individual_vehicles = False` to not collect all individual vehicles' data at each time step when not needed.

The `example_data.json` file shows the output of the `step_through` function.

```python
sim_data = sim.step_through(end_step=1000)                    # Run simulation until step 1000
sim_data = sim.step_through(n_steps=1000, prev_data=sim_data) # Run simulation for 1000 steps
sim_data = sim.step_through(sim_dur=1000, prev_data=sim_data) # Run simulation for 1000s
```

Use `sim.get_vehicle_data()` and `sim.get_all_vehicle_data()` to get individual/all vehicles' data during the simulation. `sim.get_last_step_vehicles()` can also be used to get the list of vehicle IDs of those that passed over specified (or all) detectors.

```python
bike = sim.get_vehicle_data('bike1')
all_cars = sim.get_all_vehicle_data(types=['cars'])
northbound_cars = sim.get_last_step_vehicles(detector_list=['nbound'])
```

Use `sim.is_running()` to test if the simulation has ended and run until it has, according to the simulation end time in the SUMO config file. This will call `sim.end()`, but this can also be done manually at any other point.

```python
while sim.is_running():
    sim_data = sim.step_through(prev_data=sim_data)

sim.end()
```