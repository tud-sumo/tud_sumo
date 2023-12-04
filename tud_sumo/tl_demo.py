from simulation import *
from plot import *

if __name__ == "__main__":

    # Initialise simulation
    sim = Simulation("tl_demo")
    sim.start("sumocfg file", gui=True)

    # Start tracking junction (optional for controlling tls, but required for data collection)
    sim.start_junc_tracking({"tl_id": {'flow_params': {'inflow_detectors': ["inflow_detector_1", ...], 'outflow_detectors': ["inflow_detector_2", ...], 'flow_vtypes': ["vehicle_type_1"]}}})

    while sim.is_running() and sim.curr_step < 1000:
        
        simulation_data = sim.step_through()

        if sim.curr_step % 120 == 0:
            # junc_phases = {"tl_id": {phases: [phase_0, phase_1], times: [time_0, time_1]}, ...}
            # where phases are light colour strings and times are phase length
            sim.set_phases(junc_phases)
    
            # or set_tl_colour to set indefinitely
            sim.set_tl_colour("tl_id", "colour string, eg. 'GGGGRRRRYYYY")

    # End sim
    sim.end()
    sim.save_data("tl_demo.json")

    plt = Plotter(sim)

    # Plot TL sequence
    plt.plot_tl_colours("tl_id")

    # Plot inflow/outflow
    plt.plot_junc_flows("tl_id")