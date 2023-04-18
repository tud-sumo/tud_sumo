# TU Delft SUMO Wrapper

Python SUMO wrapper, using traci.

## Requirements 

Required packages are: `tqdm`, `matplotlib` and `traci`.

## Usage example

```python    
sim = Simulation()

# The phase dictionary is a dictionary containing the phases and times
# for a given junction, eg:

# {'junctionID': {'phases': ['gggggggggggggggrrrrrrrrrrrrrrrrr', ... ],   -> Phase light settings
#                 'times':  [6.224149003633258, 2.775850996366742 ... ]}} -> Phase durations
sim.set_phases(phase_dictionary)
sim.start("files/_____.sumocfg", gui=True)

sim_data = sim.step_to(end_step=3600)                    # End siulation at step 3600
sim_data = sim.step_to(n_steps=3600, prev_data=sim_data) # Run for 3600 steps (use prev_data to append)

sim.end()
```