# TU Delft SUMO Wrapper

Python SUMO wrapper, using traci, written for AIM Lab at TU Delft. Download with [pip](https://pypi.org/project/tud-sumo/) or from [GitHub](https://github.com/calluume/tud_sumo).

## Requirements 

Required packages are: `tqdm`, `matplotlib` and `traci`.

## Updates

This is version '1.3.0'. The changelog is:
  - Added Junction tracking
  - Added Plotter objects
  - Added better SUMO vType handling
  - `sim_data` is automatically appended and stored in the `Simulation` object, instead of using `prev_data`
  - Removed default junctions

Check changes and previous versions through the project's [GitHub repository](https://github.com/calluume/tud_sumo).

## Simulation Class

```python    
from tud_sumo import tud_sumo

sim = tud_sumo.Simulation(all_vtypes=list_of_vehicle_types)
sim.start("files/scenario.sumocfg", gui=True)
```

The phase dictionary is a dictionary containing the phases and times. `math.inf` will set a light permanently, and `'-'` will keep that movement on the same setting as the previous phase. `sim.set_phases()` will then start these settings within the simulation on the next time step. If no phase dictionary is given, the simulation will use the default SUMO logic.

```python
phase_dictionary = {'junctionID': {'phases': [ 'ggrryy', 'rryygg', 'yyggrr', 'rrrr--' ], # Phase light settings
                                   'times':  [ 10.12345, 10.12345, 10.12345, math.inf ]}} # Phase durations

sim.set_phases(phase_dictionary)
```

Use `sim.step_through()` to run through the simulation, collecting and aggregating the simulation data. This will return a dictionary containing all aggregated data collected. This will contain all detector and vehicle information, although detectors and vehicle types can be specified using `detector_list` and `vTypes` respectively. Data from calls to `sim.step_through()` is automatically aggregated and appended together, although to collect data independent from other runs, set `append_data` to false. Note, this will reset the data stored in the `Simulation` object.

To improve performance, set `sim.get_individual_vehicles = False` to not collect all individual vehicles' data at each time step when not needed.

The `example_data.json` file shows the output of the `step_through` function, which can also be saved as a JSON file using `sim.save_data()`.

```python
sim_data = sim.step_through(end_step=1000) # Run simulation until step 1000
sim_data = sim.step_through(n_steps=1000)  # Run simulation for 1000 steps
sim_data = sim.step_through(sim_dur=1000)  # Run simulation for 1000s
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

sim.save_data("simulation_data.json")
```

## Junction Class

It is now possible to track specific junctions and collect vehicle and traffic light data using the `Junction` class. This is started using the `sim.start_junc_tracking()` function. By default, all junctions with traffic lights are tracked, although a list of junction IDs can be given.

The traffic light status at a junction is automatically tracked when one is present, but to track traffic flow, use a `junc_params` dictionary to define inflow and outflow detectors. The type of vehicles being tracked can also be specified using `"flow_vtypes"` in the dictionary.

```python
junc_params = {'junc_id': {
                "inflow_detectors":  ["in_1", "in_2"],
                "outflow_detectors": ["out_1", "out_2"],
                "flow_vtypes":       ["cars", "bikes"]}}
sim.start_junc_tracking(junc_params)
```

## Plotter Class

The `Plotter` class is used to visualise the simulation data. Initialise the `Plotter` object with either the current `Simulation` object, `sim_data` dictionary or the filepath to a previously saved `sim_data` file.

```python
plotter = Plotter(sim)

plotter.plot_vehicle_data("vehicle")

plotter.plot_junc_flows('J0')
plotter.plot_tl_colours('J0')
```

Both `plotter.plot_junc_flows()` and `plotter.plot_tl_colours()` both require junction tracking during the simulation.