
from random import randint
import sys
from tqdm import tqdm

sys.path.insert(0, '..')
from simulation import Simulation
from plot import Plotter

if __name__ == "__main__":

    # Initialise the simulation object.
    my_sim = Simulation(scenario_name="A20_ITCS", scenario_desc="Example traffic controllers, with a ramp meter, VSL controller and route guidance.")

    # Start the simulation, defining the sumo config files.
    my_sim.start("../../dev/sumo_scenarios/a20/a20.sumocfg", get_individual_vehicle_data=False, gui="-gui" in sys.argv)

    # Add a tracked junction to the intersection with ID "utsc", which will track signal phases/times.
    my_sim.add_tracked_junctions(["utsc"])

    # Set traffic signal phases. The junc_phases dict can be used for multiple junctions.
    my_sim.set_phases({"utsc": {"phases": ["GGrr", "yyrr", "rrGG", "rryy"], "times": [27, 3, 17, 3]}})

    # Add a ramp meter to the junction with ID "crooswijk_meter". The junc_params dict can be used to
    # define meter specifc parameters (min/max rate, spillback tracking or queue detectors) and flow specific
    # parameters (inflow/outflow detectors used to calculate in/out flow).
    my_sim.add_tracked_junctions({"crooswijk_meter": {'meter_params': {'min_rate': 200, 'max_rate': 2000, 'queue_detector': "cw_ramp_queue"},
                                                    'flow_params': {'inflow_detectors': ["cw_ramp_inflow", "cw_rm_upstream"], 'outflow_detectors': ["cw_rm_downstream"]}}})
    
    # Add a Route Guidance (RG) controller. RG controllers need a detector or edge that will act as the 
    # redirection point and a target edge/route ID to redirect drivers to. Can also define a diversion
    # percent to randomly divert only a percent of drivers, and a highlight colour for the SUMO gui, which
    # will highlight affected drivers.
    my_sim.add_controllers({"rerouter": {"type": "RG", "detector_ids": ["rerouter_2"], "new_destination": "urban_out", "diversion_pct": 0.25, "highlight": "00FF00"}})

    # Add tracked edges. This will track some basic information, such as average speed etc, but can also
    # be used to create space-time diagrams as individual vehicle speeds and positions are tracked.
    my_sim.add_tracked_edges(["126730026", "1191885773", "1191885771", "126730171", "1191885772", "948542172", "70944365", "308977078", "1192621075"])

    # Add scheduled events from a JSON file (can be dictionary). Use the format as in example_incident.json
    my_sim.add_events("example_incident.json")

    n, sim_dur = 50, 500
    pbar = tqdm(desc="Running sim (step 0, 0 vehs)", total=sim_dur)
    while my_sim.curr_step < sim_dur:

        # Set ramp metering rate.
        my_sim.set_tl_metering_rate("crooswijk_meter", randint(1200, 2000))
        
        # Step through n steps.
        my_sim.step_through(n, pbar=pbar)

        if my_sim.curr_step == 250:
            # Activate RG controller & update UTSC phases.
            my_sim.controllers["rerouter"].activate()
            my_sim.set_phases({"utsc": {"phases": ["GGrr", "yyrr", "rrGG", "rryy"], "times": [37, 3, 7, 3]}}, overwrite=False)

        # Deactivate RG controller.
        if my_sim.curr_step == 450: my_sim.controllers["rerouter"].deactivate()

    # End the simulation.
    my_sim.end()

    # Save the simulation data & print a summary, which is also saved.
    my_sim.save_data("example_data.json")
    my_sim.print_summary(save_file="example_summary.txt")
    