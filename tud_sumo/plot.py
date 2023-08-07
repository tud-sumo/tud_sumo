import json, matplotlib.pyplot as plt, numpy as np
import os.path, sys
from copy import deepcopy
from tud_sumo import *

class Plotter:
    def __init__(self, simulation):
        """
        :param simulation: Either simulation object, sim_data dict or sim_data filepath.
        """

        self.simulation = None
        if isinstance(simulation, Simulation):
            self.simulation = simulation
            self.sim_data = simulation.all_data

        elif isinstance(simulation, str):

            if os.path.exists(simulation):
                with open(simulation, "r") as fp:
                    self.sim_data = json.load(fp)
            else: raise FileNotFoundError("Plotter.init: simulation file '{0}' not found".format(simulation))

        elif isinstance(simulation, dict): self.sim_data = simulation

        else: raise TypeError("Plotter.init: Invalid simulation type (must be Simulation|str|dict, not '{0}')".format(type(simulation).__name__))

    def plot_junc_flows(self, junc_id, vtypes=None, plot_n_vehicles=False, plot_all=True, save_fig=None):
        """
        Plot junction flow, either as inflow & outflow or number of vehicles at the intersection.
        :param junc_id: Junction ID
        :param vtypes: vType flows
        :param plot_n_vehicles: If true, plot number of vehicles in the intersection, else inflow and outflow
        :param plot_all: If true, plot total values as well as vType data
        :param save_fig: Output image filename, will show image if not given
        """

        if self.simulation != None:

            if junc_id in self.simulation.tracked_juncs.keys(): tl = self.simulation.tracked_juncs[junc_id]
            else: raise KeyError("Plotter.plot_junc_flows: Junction '{0}' not found in tracked junctions".format(junc_id))

            if tl.track_flow: junc_flows = deepcopy(tl.get_curr_data()["flows"])
            else: raise ValueError("Plotter.plot_junc_flows: No traffic light at junction '{0}'".format(junc_id))

            start_time, end_time = tl.init_time, tl.curr_time
            sim_step_len = self.simulation.step_length

        elif junc_id in self.sim_data["data"]["junctions"].keys():
            if "flows" in self.sim_data["data"]["junctions"][junc_id].keys():
                junc_flows = deepcopy(self.sim_data["data"]["junctions"][junc_id]["flows"])
                start_time, end_time = self.sim_data["data"]["junctions"][junc_id]["init_time"], self.sim_data["data"]["junctions"][junc_id]["curr_time"]
                sim_step_len = self.sim_data["step_len"]

            else: raise ValueError("Plotter.plot_junc_flows: Junction '{0}' does not track flows (no detectors)".format(junc_id))
        else: raise KeyError("Plotter.plot_junc_flows: Junction '{0}' not found in tracked junctions".format(junc_id))

        if vtypes == None: vtypes = list(junc_flows["avg_inflows"].keys())
        if "all" in vtypes and not plot_all: vtypes.remove("all")

        fig, ax = plt.subplots()

        time_steps = np.arange(start_time, end_time+sim_step_len, sim_step_len)

        for vtype in vtypes:
            inflow_data, outflow_data = junc_flows["all_inflows"][vtype], junc_flows["all_outflows"][vtype]
            cumulative_inflow, cumulative_outflow = np.cumsum(inflow_data), np.cumsum(outflow_data)
            throughput = np.subtract(cumulative_inflow, cumulative_outflow)

            t_inflow, t_outflow = np.sum(inflow_data), np.sum(outflow_data)

            if t_inflow != t_outflow:
                print("Plotter.plot_tl_flows: Total inflow ({0}) does not match total outflow ({1})".format(t_inflow, t_outflow))

            if not plot_n_vehicles:
                plt.title("Vehicle Inflow & Outflow at Intersection '{0}'".format(junc_id))
                plt.ylabel("No. of Vehicles")
                inflow_line = plt.plot(time_steps, cumulative_inflow, label=vtype+' in', linewidth=2)
                plt.plot(time_steps, cumulative_outflow, label=vtype + ' out', linestyle='--', linewidth=1, color=inflow_line[-1].get_color())
            else:
                plt.title("Number of Vehicles in Intersection '{0}'".format(junc_id))
                plt.ylabel("No. of Vehicles")
                plt.plot(time_steps, throughput, label=vtype, linewidth=2)

        plt.xlim([time_steps[0], time_steps[-1]])
        plt.ylim(bottom=0)
        plt.xlabel("Time (s)")
        plt.tight_layout()
        ax.legend(title="Types", fontsize="small")

        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)
                
    
    def plot_tl_colours(self, tl_id, time_range=None, plot_percent=False, save_fig=None):
        """
        Plot traffic light sequence, as colours or green/red/yellow durations as a percent of time.
        :param tl_id: Traffic light ID
        :param time_range: Time range to plot
        :param plot_percent: Denotes whether to plot colours as percent of time
        :param save_fig: Output image filename, will show image if not given
        """

        plt_colour = {"G": "green", "Y": "yellow", "R": "red"}

        if self.simulation != None:

            if tl_id in self.simulation.tracked_juncs.keys(): tl = self.simulation.tracked_juncs[tl_id]
            else: raise KeyError("Plotter.plot_tl_colours: Junction '{0}' not found in tracked junctions".format(tl_id))

            if tl.has_tl: tl_durs = deepcopy(tl.durations)
            else: raise ValueError("Plotter.plot_tl_colours: No traffic light at junction '{0}'".format(tl_id))

            m_len = tl.m_len
            init_time = tl.init_time

        elif tl_id in self.sim_data["data"]["junctions"].keys():
            if "tl" in self.sim_data["data"]["junctions"][tl_id].keys():
                tl_durs = deepcopy(self.sim_data["data"]["junctions"][tl_id]["tl"]["m_phases"])
                m_len, init_time = self.sim_data["data"]["junctions"][tl_id]["tl"]["m_len"], self.sim_data["data"]["junctions"][tl_id]["init_time"]

            else: raise ValueError("Plotter.plot_tl_colours: No traffic light at junction '{0}'".format(tl_id))
        else: raise KeyError("Plotter.plot_tl_colours: Junction '{0}' not found in tracked junctions".format(tl_id))

        
        if (isinstance(time_range, list) or isinstance(time_range, tuple)) and time_range != None:
            if len(time_range) != 2: raise ValueError("Plotter.plot_tl_colours: Invalid time range (must have length 2, not {0})".format(len(time_range)))
            elif time_range[0] >= time_range[1]: raise ValueError("Plotter.plot_tl_colours: Invalid time range (start_time ({0}) >= end_time ({1}))".format(start_time, end_time))
            else:
                clipped_tl_durs = []
                start_time, end_time = time_range[0], time_range[1]
                for m in tl_durs:
                    phase_times, phase_colours = [c_dur for (_, c_dur) in m], [colour for (colour, _) in m]
                    cum_phase_times = list(np.cumsum(phase_times))
                    if start_time < 0 or end_time > cum_phase_times[-1]:
                        raise ValueError("Plotter.plot_tl_colours: Invalid time range (values [{0}-{1}] must be in range [0-{2}])".format(start_time, end_time, cum_phase_times[-1]))

                    times_in_range = [time >= start_time and time <= end_time for time in cum_phase_times]
                    
                    if True in times_in_range:
                        start_phase, end_phase = np.where(times_in_range)[0][0], np.where(times_in_range)[0][-1]
                        new_m_dur = [[phase_colours[i], phase_times[i]] for i in range(start_phase, end_phase + 1)]
                        
                        new_m_dur[0][1] = new_m_dur[0][1] + cum_phase_times[start_phase-1] - start_time if start_phase >= 1 else new_m_dur[0][1] - start_time
                        
                        # Add end buffer for last phase
                        if end_phase < len(cum_phase_times) - 1 and end_time - cum_phase_times[end_phase] > 0:
                            new_m_dur.append([phase_colours[end_phase + 1], end_time - cum_phase_times[end_phase]])
                        
                    else:
                        # If start_time and end_time both in same phase
                        times_after_start = [time >= start_time for time in cum_phase_times]
                        start_phase = np.where(times_after_start)[0][0]
                        new_m_dur = [[phase_colours[start_phase], end_time - start_time]]

                    clipped_tl_durs.append(new_m_dur)
                
                tl_durs = clipped_tl_durs

        fig, ax = plt.subplots()

        if plot_percent:
            percent_tl_durs = [[] for _ in range(m_len)]
            for idx, m_durs in enumerate(tl_durs):
                total_len = sum([x[1] for x in m_durs])
                percent_tl_durs[idx].append(['G', sum([x[1] for x in m_durs if x[0] == 'G']) / total_len * 100])
                percent_tl_durs[idx].append(['Y', sum([x[1] for x in m_durs if x[0] == 'Y']) / total_len * 100])
                percent_tl_durs[idx].append(['R', sum([x[1] for x in m_durs if x[0] == 'R']) / total_len * 100])

            tl_durs = percent_tl_durs
            plt.xlabel("Movements")
            plt.ylabel("Colour Duration (%)")
            plt.ylim((0, 100))
        else:
            plt.xlabel("Time (s)")
            plt.ylabel("Movement")

        ms = list([str(i) for i in range(1, m_len + 1)])

        curr_colour = 'G'
        all_plotted = False

        if time_range != None: offset_value = time_range[0]
        else: offset_value = init_time
        offset = [offset_value for _ in range(m_len)]

        while not all_plotted:

            curr_bar = [0 for _ in range(m_len)]
            for idx, m_durs in enumerate(tl_durs):
                if len(m_durs) == 0: continue
                
                if m_durs[0][0] == curr_colour:
                    curr_bar[idx] = m_durs[0][1]
                    tl_durs[idx].pop(0)
            
            all_plotted = True
            for m_durs in tl_durs:
                if len(m_durs) != 0: all_plotted = False

            if plot_percent: plt.bar(ms, curr_bar, bottom=offset, color=plt_colour[curr_colour])
            else: plt.barh(ms, curr_bar, left=offset, color=plt_colour[curr_colour])

            for m in range(m_len): offset[m] = offset[m] + curr_bar[m]

            if curr_colour == 'G': curr_colour = 'Y'
            elif curr_colour == 'Y': curr_colour = 'R'
            elif curr_colour == 'R': curr_colour = 'G'

        plt.title("Light Phase Durations")
        
        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)

    def plot_vehicle_data(self, dataset, save_fig=None):
        """
        Plots all collected vehicle or detector data.
        :param dataset: Dataset key (either "vehicle" or ["detector", detector_id])
        :param save_fig: Output image filename, will show image if not given
        """

        default_labels = {"no_vehicles": "No. of Vehicles", "tts": "TTS (s)", "delay": "Delay (s)",
                        "speeds": "Avg. Speed (km/h)", "veh_counts": "No. of Vehicles", "occupancies": "Occupancy (%)"}

        if isinstance(dataset, str): # Sim data
            if dataset in self.sim_data["data"].keys(): data = self.sim_data["data"][dataset]
            else: raise KeyError("Plotter.plot_vehicle_data: Unrecognised dataset key '{0}'".format(dataset))
            title = dataset.title() + " Data"
        elif isinstance(dataset, list) or isinstance(dataset, tuple): # Detector data
            data = self.sim_data["data"]
            for key in dataset:
                if key in data.keys(): data = data[key]
                else: raise KeyError("Plotter.plot_vehicle_data: Unrecognised dataset key '{0}'".format(dataset[-1]))
            title = ': '.join(dataset)

        else: raise TypeError("Plotter.plot_vehicle_data: Invalid dataset key type (must be [int|str], not '{0}')".format(type(dataset).__name__))

        plot_data = {key: data[key] for key in data if key in default_labels and len(data[key]) != 0}
        fig, axes = plt.subplots(len(plot_data))
        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        for idx, (ax, (data_key, data_vals)) in enumerate(zip(axes, plot_data.items())):
            if not isinstance(data_vals, list): continue
            ax.plot([x * step for x in range(start, end)], data_vals, zorder=3)
            ax.set_title(data_key)
            if data_key in default_labels.keys(): ax.set_ylabel(default_labels[data_key])
            if idx < len(axes) - 1: ax.tick_params('x', labelbottom=False)
            else: ax.set_xlabel('Time (s)')
            ax.set_xlim([start * step, (end - 1) * step])
            ax.set_ylim([0, max(data_vals) * 1.05])
            ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

        fig.suptitle(title)
        fig.tight_layout()

        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)