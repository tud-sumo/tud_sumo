from simulation import Simulation
from plot import Plotter
import sys

def get_avg_occ(simulation, ids, d_interval):
    occupancy_vals = [simulation.get_interval_detector_data(d_id, d_interval, "occupancies", True) for d_id in ids]
    return sum(occupancy_vals) / len(occupancy_vals)
    
if __name__ == "__main__":

    sim = Simulation("demo")
    sim.start('../dev/scenarios/a20/a20.sumocfg',
              cmd_options=["--extrapolate-departpos", "--eager-insert", "--emergency-insert", "-H"],
              get_individual_vehicle_data=False, gui='-gui' in sys.argv)
    
    downstream_detectors = ["cw_down_occ_0", "cw_down_occ_1"]

    min_r, max_r = 200, 2000

    curr_r, k_r = max_r, 70
    curr_occ = None
    prev_occ = None
    des_occ = 10

    meter_id = "crooswijk_meter"
    downstream_detectors = ["cw_down_occ_0", "cw_down_occ_1"]

    sim.start_junc_tracking({meter_id: {'meter_params': {'min_rate': min_r, 'max_rate': max_r, 'queue_detector': 'cw_ramp_queue'}}})

    controllers = sim.add_controllers({'rg_0': {'type': 2, 'diversion_pct': 1, 'target': 'reroute_test', 'detector_ids': 'rerouter_2', 'highlight': '#00FF00'},
                                       'vsl_0': {'type': 1, 'geometry_ids': ['321901470', '126729982']}})

    controllers['rg_0'].activate()
    controllers['vsl_0'].set_limit(70)
    sim.add_events("../dev/scenarios/a20/bottleneck.json")
    sim.start_edge_tracking(['321901470', '126729982', '126730069', '126730059', '509506847'])
    interval, warmup = 60, 120

    while sim.is_running() and sim.curr_step < 2200:
        if sim.curr_step == 20: sim.set_tl_metering_rate(meter_id, 1200)
        sim.step_through()
        
        
        if sim.curr_step == warmup - interval: prev_occ = get_avg_occ(sim, downstream_detectors, interval)
        if sim.curr_step == 200: sim.reset_data()

        if sim.curr_step == 250: controllers['rg_0'].activate()
        elif sim.curr_step == 750: controllers['rg_0'].deactivate()
        elif sim.curr_step == 1800: controllers['rg_0'].activate()

        if sim.curr_step == 400: controllers['vsl_0'].set_limit(50)
        elif sim.curr_step == 600: controllers['vsl_0'].set_limit(40)
        elif sim.curr_step == 720: controllers['vsl_0'].deactivate()
        elif sim.curr_step == 1200: controllers['vsl_0'].set_limit(60)
        elif sim.curr_step == 1500: controllers['vsl_0'].set_limit(40)
        elif sim.curr_step == 1620: controllers['vsl_0'].deactivate()

        if sim.curr_step >= warmup and (sim.curr_step - warmup) % interval == 0:
            curr_occ = get_avg_occ(sim, downstream_detectors, interval)
    
            curr_r = curr_r + (k_r*(des_occ - prev_occ))
            curr_r = min(max_r, max(min_r, curr_r))
            sim.set_tl_metering_rate(meter_id, curr_r, 2)

            prev_occ = curr_occ

    sim.end()

    sim.save_data("controller_demo.json")

    plotter = Plotter("controller_demo.json")

    plotter.plot_space_time_diagram(['321901470', '126729982', '126730069', '126730059', '509506847'])
    plotter.plot_vsl_data('vsl_0')
    plotter.plot_rg_data('rg_0')
    plotter.plot_metering_rate("crooswijk_meter")
    