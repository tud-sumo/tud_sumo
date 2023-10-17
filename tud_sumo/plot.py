import json, matplotlib.pyplot as plt, numpy as np
import os.path
from copy import deepcopy
from simulation import Simulation

class Plotter:
    def __init__(self, simulation, sim_label=None):
        """
        :param simulation: Either simulation object, sim_data dict or sim_data filepath
        :param sim_label:  Simulation or scenario label added to the beginning of all plot titles
        """

        self.simulation = None
        self.sim_label = sim_label + ": " if sim_label != None else ""
        if isinstance(simulation, Simulation):
            self.simulation = simulation
            self.sim_data = simulation.all_data
            self.units = simulation.units.name

        elif isinstance(simulation, str):

            if os.path.exists(simulation):
                with open(simulation, "r") as fp:
                    self.sim_data = json.load(fp)
                    self.units = self.sim_data["units"]
            else: raise FileNotFoundError("Plot.init: Simulation file '{0}' not found.".format(simulation))

        elif isinstance(simulation, dict): self.sim_data, self.units = simulation, simulation["units"]

        else: raise TypeError("Plot.init: Invalid simulation type (must be Simulation|str|dict, not '{0}').".format(type(simulation).__name__))

        self.default_labels = {"no_vehicles": "No. of Vehicles", "tts": "TTS (s)", "delay": "Delay (s)",
                               "veh_counts": "No. of Vehicles", "occupancies": "Occupancy (%)"}
        match self.units:
            case "METRIC": self.default_labels["speeds"] = "Avg. Speed (km/h)"
            case "IMPERIAL": self.default_labels["speeds"] = "Avg. Speed (mph)"
            case "UK": self.default_labels["speeds"] = "Avg. Speed (mph)"
            case _: raise ValueError("Plot.init: Invalid simulation units '{0}' (must be 'METRIC'|'IMPERIAL'|'UK').".format(self.units.upper()))
        
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
            else: raise KeyError("Plot.plot_junc_flows: Junction '{0}' not found in tracked junctions.".format(junc_id))

            if tl.track_flow: junc_flows = deepcopy(tl.get_curr_data()["flows"])
            else: raise ValueError("Plot.plot_junc_flows: No traffic light at junction '{0}'.".format(junc_id))

            start_time, end_time = tl.init_time, tl.curr_time
            sim_step_len = self.simulation.step_length

        elif junc_id in self.sim_data["data"]["junctions"].keys():
            if "flows" in self.sim_data["data"]["junctions"][junc_id].keys():
                junc_flows = deepcopy(self.sim_data["data"]["junctions"][junc_id]["flows"])
                start_time, end_time = self.sim_data["data"]["junctions"][junc_id]["init_time"], self.sim_data["data"]["junctions"][junc_id]["curr_time"]
                sim_step_len = self.sim_data["step_len"]

            else: raise ValueError("Plot.plot_junc_flows: Junction '{0}' does not track flows (no detectors).".format(junc_id))
        else: raise KeyError("Plot.plot_junc_flows: Junction '{0}' not found in tracked junctions.".format(junc_id))

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
                print("Plotter.plot_tl_flows: Total inflow ({0}) does not match total outflow ({1}).".format(t_inflow, t_outflow))

            if not plot_n_vehicles:
                plt.title(self.sim_label+"Vehicle Inflow & Outflow at Intersection '{0}'".format(junc_id))
                plt.ylabel("No. of Vehicles")
                inflow_line = plt.plot(time_steps, cumulative_inflow, label=vtype+' in', linewidth=2)
                plt.plot(time_steps, cumulative_outflow, label=vtype + ' out', linestyle='--', linewidth=1, color=inflow_line[-1].get_color())
            else:
                plt.title(self.sim_label+"Number of Vehicles in Intersection '{0}'".format(junc_id))
                plt.ylabel("No. of Vehicles")
                plt.plot(time_steps, throughput, label=vtype, linewidth=2)

        plt.xlim([time_steps[0], time_steps[-1]])
        plt.ylim(bottom=0)
        plt.xlabel("Simulation Time (s)")
        plt.tight_layout()
        ax.legend(title="Types", fontsize="small", shadow=True)

        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)
                
    
    def plot_tl_colours(self, tl_id, plt_movements=None, time_range=None, plot_percent=False, save_fig=None):
        """
        Plot traffic light sequence, as colours or green/red/yellow durations as a percent of time.
        :param tl_id: Traffic light ID
        :param plt_movements: List of movements to plot by index (defaults to all)
        :param time_range: Time range to plot
        :param plot_percent: Denotes whether to plot colours as percent of time
        :param save_fig: Output image filename, will show image if not given
        """

        plt_colour = {"G": "green", "Y": "yellow", "R": "red"}

        if self.simulation != None:

            if tl_id in self.simulation.tracked_juncs.keys(): tl = self.simulation.tracked_juncs[tl_id]
            else: raise KeyError("Plot.plot_tl_colours: Junction '{0}' not found in tracked junctions.".format(tl_id))

            if tl.has_tl: tl_durs = deepcopy(tl.durations)
            else: raise ValueError("Plot.plot_tl_colours: No traffic light at junction '{0}'.".format(tl_id))

            m_len = tl.m_len
            init_time = tl.init_time

        elif tl_id in self.sim_data["data"]["junctions"].keys():
            if "tl" in self.sim_data["data"]["junctions"][tl_id].keys():
                tl_durs = deepcopy(self.sim_data["data"]["junctions"][tl_id]["tl"]["m_phases"])
                m_len, init_time = self.sim_data["data"]["junctions"][tl_id]["tl"]["m_len"], self.sim_data["data"]["junctions"][tl_id]["init_time"]

            else: raise ValueError("Plot.plot_tl_colours: No traffic light at junction '{0}'.".format(tl_id))
        else: raise KeyError("Plot.plot_tl_colours: Junction '{0}' not found in tracked junctions.".format(tl_id))

        if plt_movements != None:
            m_mask = plt_movements
            m_mask.sort()
            for idx in m_mask:
                if idx >= m_len or idx < 0: raise ValueError("Plot.plot_tl_colours: Invalid movement index '{0}' (must be 0 <= idx <= {1})".format(idx, m_len - 1))
            for i in reversed(range(m_len)):
                if i not in m_mask: tl_durs.pop(i)

            m_len = len(m_mask)

        xlim = self.sim_data["end"] * self.sim_data["step_len"]
        if isinstance(time_range, (list, tuple)) and time_range != None:
            if len(time_range) != 2: raise ValueError("Plot.plot_tl_colours: Invalid time range (must have length 2, not {0}).".format(len(time_range)))
            elif time_range[0] >= time_range[1]: raise ValueError("Plot.plot_tl_colours: Invalid time range (start_time ({0}) >= end_time ({1})).".format(start_time, end_time))
            else:
                clipped_tl_durs = []
                start_time, end_time = time_range[0], time_range[1]
                xlim = end_time * self.sim_data["step_len"]
                for m in tl_durs:
                    phase_times, phase_colours = [c_dur for (_, c_dur) in m], [colour for (colour, _) in m]
                    cum_phase_times = list(np.cumsum(phase_times))
                    if start_time < 0 or end_time > cum_phase_times[-1]:
                        raise ValueError("Plot.plot_tl_colours: Invalid time range (values [{0}-{1}] must be in range [0-{2}]).".format(start_time, end_time, cum_phase_times[-1]))

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
            plt.xlabel("Simulation Time (s)")
            plt.ylabel("Movement")
            plt.xlim(0, xlim)

        if plt_movements == None: ms = list([str(i) for i in range(1, m_len + 1)])
        else: ms = list([str(i) for i in m_mask])

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

        plt.title(self.sim_label+"Light Phase Durations")
        
        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)

    def plot_metering_rate(self, tl_id, show_events=True, save_fig=None):

        if self.simulation != None:

            if tl_id in self.simulation.tracked_juncs.keys(): tl = self.simulation.tracked_juncs[tl_id]
            else: raise KeyError("Plot.plot_metering_rate: Junction '{0}' not found in tracked junctions.".format(tl_id))

            if tl.is_meter:
                metering_rates = tl.metering_rates
                rate_times = tl.rate_times
                min_r, max_r = tl.min_rate, tl.max_rate
            else: raise ValueError("Plot.plot_metering_rate: Junction '{0}' is not tracked as a meter.".format(tl_id))

        elif tl_id in self.sim_data["data"]["junctions"].keys():
            if "meter" in self.sim_data["data"]["junctions"][tl_id].keys():
                metering_rates = self.sim_data["data"]["junctions"][tl_id]["meter"]["metering_rates"]
                rate_times = self.sim_data["data"]["junctions"][tl_id]["meter"]["rate_times"]
                min_r, max_r = self.sim_data["data"]["junctions"][tl_id]["meter"]["min_rate"], self.sim_data["data"]["junctions"][tl_id]["meter"]["max_rate"]

            else: raise ValueError("Plot.plot_metering_rate: Junction '{0}' is not tracked as a meter.".format(tl_id))
        else: raise KeyError("Plot.plot_metering_rate: Junction '{0}' not found in tracked junctions.".format(tl_id))

        fig, ax = plt.subplots(1, 1)
        ax.plot([rate_time * self.sim_data["step_len"] for rate_time in rate_times], metering_rates, label="Metering Rate", linewidth=1.5, zorder=3)

        ax.axhline(max_r, label="Minimum / Maximum Rate", color="green", linestyle="--", alpha=0.5, zorder=1)
        ax.axhline(min_r, color="green", linestyle="--", alpha=0.5, zorder=2)
        ax.set_xlim([self.sim_data["start"] * self.sim_data["step_len"], self.sim_data["end"] * self.sim_data["step_len"]])
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)
        ax.set_ylabel("Metering Rate (veh/hr)")
        ax.set_xlabel("Simulation Time (s)")
        ax.set_title("{0}'{1}' Metering Rate".format(self.sim_label, tl_id), pad=15)
        box = ax.get_position()
        ax.set_position([box.x0, box.y0 + box.height * 0.08,
                        box.width, box.height * 0.92])

        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13),
          fancybox=True, ncol=2)
        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)

        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)

    def plot_meter_queue_length(self, tl_id, show_events=True, save_fig=None):

        if self.simulation != None:

            if tl_id in self.simulation.tracked_juncs.keys(): tl = self.simulation.tracked_juncs[tl_id]
            else: raise KeyError("Plot.plot_metering_rate: Junction '{0}' not found in tracked junctions.".format(tl_id))

            if tl.is_meter: 
                if tl.measure_queues:
                    queue_lengths = tl.queue_lengths
                    queue_delays = tl.queue_delays
                else: raise ValueError("Plot.plot_metering_rate: Meter '{0}' does not track queue lengths (no queue detector).".format(tl_id))
            else: raise ValueError("Plot.plot_metering_rate: Junction '{0}' is not tracked as a meter.".format(tl_id))

        elif tl_id in self.sim_data["data"]["junctions"].keys():
            if "meter" in self.sim_data["data"]["junctions"][tl_id].keys():
                if "queue_lengths" in self.sim_data["data"]["junctions"][tl_id]["meter"].keys():
                    queue_lengths = self.sim_data["data"]["junctions"][tl_id]["meter"]["queue_lengths"]
                    queue_delays = self.sim_data["data"]["junctions"][tl_id]["meter"]["queue_delays"]

                else: raise ValueError("Plot.plot_metering_rate: Meter '{0}' does not track queue lengths (no queue detector).".format(tl_id))
            else: raise ValueError("Plot.plot_metering_rate: Junction '{0}' is not tracked as a meter.".format(tl_id))
        else: raise KeyError("Plot.plot_metering_rate: Junction '{0}' not found in tracked junctions.".format(tl_id))

        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]
        fig, ax1 = plt.subplots(1, 1)

        colour = 'tab:blue'
        ax1.plot([x * step for x in range(start, end)], queue_lengths, linewidth=1.5, zorder=3, color=colour)
        ax1.set_ylabel("Number of Vehicles", color=colour)
        ax1.set_xlabel("Simulation Time (s)")
        ax1.tick_params(axis='y', labelcolor=colour)
        ax1.set_xlim([0, end * step])
        ax1.set_ylim([0, max(queue_lengths) * 1.05])

        ax2 = ax1.twinx()

        colour = 'tab:red'
        queue_delays = get_cumulative_arr(queue_delays)
        ax2.plot([x * step for x in range(start, end)], queue_delays, linewidth=1.5, zorder=2, color=colour)
        ax2.set_ylabel("Queue Delay (s)", color=colour)
        ax2.tick_params(axis='y', labelcolor=colour)
        ax2.set_ylim([0, max(queue_delays) * 1.05])

        fig.suptitle("{0}'{1}' Queue Lengths & Cumulative Delay".format(self.sim_label, tl_id))

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax1)

        fig.tight_layout()

        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)

    def plot_vehicle_detector_data(self, dataset, save_fig=None):
        """
        Plots all collected vehicle or detector data.
        :param dataset: Dataset key (either "vehicle" or ["detector", detector_id])
        :param save_fig: Output image filename, will show image if not given
        """

        if isinstance(dataset, str): # Sim data
            if dataset in self.sim_data["data"].keys(): data = self.sim_data["data"][dataset]
            else: raise KeyError("Plot.plot_vehicle_detector_data: Unrecognised dataset key '{0}'.".format(dataset))
            title = self.sim_label + dataset.title() + " Data"
        elif isinstance(dataset, list) or isinstance(dataset, tuple): # Detector data
            data = self.sim_data["data"]
            for key in dataset:
                if key in data.keys(): data = data[key]
                else: raise KeyError("Plot.plot_vehicle_detector_data: Unrecognised dataset key '{0}'.".format(dataset[-1]))
            title = self.sim_label + ': '.join(dataset)

        else: raise TypeError("Plot.plot_vehicle_detector_data: Invalid dataset key type (must be [int|str], not '{0}').".format(type(dataset).__name__))

        plot_data = {key: data[key] for key in data if key in self.default_labels and len(data[key]) != 0}
        fig, axes = plt.subplots(len(plot_data))
        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        for idx, (ax, (data_key, data_vals)) in enumerate(zip(axes, plot_data.items())):
            if not isinstance(data_vals, list): continue
            ax.plot([x * step for x in range(start, end)], data_vals, zorder=3)
            ax.set_title(data_key)
            if data_key in self.default_labels.keys(): ax.set_ylabel(self.default_labels[data_key])
            if idx < len(axes) - 1:
                if idx == 0: ax.set_title(title)
                ax.tick_params('x', labelbottom=False)
            else: ax.set_xlabel('Simulation Time (s)')
            ax.set_xlim([start * step, (end - 1) * step])
            ax.set_ylim([0, max(data_vals) * 1.05])
            ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

        fig.tight_layout()

        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)

    def plot_n_vehicles(self, cumulative=True, show_events=True, save_fig=None):
        """
        Plot the number of vehicles in the system.
        :param cumulative: Bool denoting whether values are displayed cumulatively
        :param show_events: Bool denoting whether to plot when events occur
        :param save_fig: Output image filename, will show image if not given
        """

        
        no_vehicles = self.sim_data["data"]["vehicle"]["no_vehicles"]
        data_vals = get_cumulative_arr(no_vehicles) if cumulative else no_vehicles
        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        fig, ax = plt.subplots(1, 1)

        title, xlabel = "Number of Vehicles", "No. of Vehicles"
        if cumulative:
            title = "Cumulative "+title
            xlabel = "Cumulative "+xlabel
        fig.suptitle(self.sim_label + title)

        ax.plot([x * step for x in range(start, end)], data_vals, zorder=3)
        ax.set_xlim([start * step, (end - 1) * step])
        ax.set_ylim([0, max(data_vals) * 1.05])
        ax.set_xlabel("Simulation Time (s)")
        ax.set_ylabel(xlabel)
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)
        
        fig.tight_layout()

        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)

    def plot_cumulative_curve(self, inflow_detectors=None, outflow_detectors=None, outflow_offset=0, show_events=True, save_fig=None):
        """
        Plot inflow and outflow cumulative curves, either system-wide or using inflow/outflow detectors (if given).
        :param inflow_detectors: List of inflow detectors
        :param outflow_detectors: List of outflow detectors
        :param outflow_offset: Offset for outflow values if not starting at t=0
        :param show_events: Bool denoting whether to plot events
        :param save_fig: Output image filename, will show image if not given
        """

        inflows, outflows = [], []

        if inflow_detectors == None and outflow_detectors == None:

            if "all_vehicles" not in self.sim_data["data"].keys():
                raise KeyError("Plot.plot_cumulative_curve: Plot requires 'individual_vehicle_data=True' during the simulation to plot system cumulative curves.")
            else: all_vehicle_data = self.sim_data["data"]["all_vehicles"]
            
            prev_vehicles = set([])
            
            for step, vehicles in enumerate(all_vehicle_data):
                curr_vehicles = set(vehicles.keys())
                inflows.append(len(curr_vehicles - prev_vehicles))
                outflows.append(len(prev_vehicles - curr_vehicles))
                prev_vehicles = curr_vehicles
        else:
            if inflow_detectors == None or outflow_detectors == None:
                raise TypeError("Plot.plot_cumulative_curve: If using detectors, both inflow and outflow detectors are required.")
            
            detector_data = self.sim_data["data"]["detector"]
            if not hasattr(inflow_detectors, "__iter__"): inflow_detectors = [inflow_detectors]
            if not hasattr(outflow_detectors, "__iter__"): outflow_detectors = [outflow_detectors]

            if len(set(inflow_detectors + outflow_detectors) - set(detector_data.keys())) != 0:
                raise KeyError("Plot.plot_cumulative_curve: Detectors '{0}' could not be found.".format(set(inflow_detectors + outflow_detectors) - set(detector_data.keys())))

            prev_in_vehicles, prev_out_vehicles = set([]), set([])
            for step_no in range(self.sim_data["end"] - self.sim_data["start"]):
                vehs_in, vehs_out = set([]), set([])

                for detector_id in inflow_detectors: vehs_in = vehs_in | set(detector_data[detector_id]["veh_ids"][step_no])
                for detector_id in outflow_detectors: vehs_out = vehs_out | set(detector_data[detector_id]["veh_ids"][step_no])

                inflows.append(len(vehs_in - prev_in_vehicles))
                if step_no > outflow_offset / self.sim_data["step_len"]:
                    outflows.append(len(vehs_out - prev_out_vehicles))
                else: outflows.append(0)

                prev_in_vehicles = prev_in_vehicles | vehs_in
                prev_out_vehicles = prev_out_vehicles | vehs_out

        inflows = get_cumulative_arr(inflows)
        outflows = get_cumulative_arr(outflows)
        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]
        x_vals = [x * step for x in range(start, end)]

        fig, ax = plt.subplots(1, 1)
        fig.suptitle(self.sim_label+"Cumulative Arrival-Departure Curve")
        ax.plot(x_vals, inflows, label="Inflow", zorder=3)
        ax.plot(x_vals, outflows, label="Outflow", zorder=4)
        ax.set_xlim([start * step, (end - 1) * step])
        ax.set_ylim([0, max(inflows) * 1.05])
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)
        ax.set_xlabel("Simulation Time (s)")
        ax.set_ylabel("Cumulative No. of Vehicles")
        ax.legend(loc='lower right', shadow=True)

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)

        fig.tight_layout()

        if save_fig is None: plt.show()
        else: plt.savefig(save_fig)

    def plot_event(self, ax):
        _, y_lim = ax.get_xlim(), ax.get_ylim()
        for event in self.sim_data["data"]["events"]["completed"]:
            ax.axvspan(event["start_time"] * self.sim_data["step_len"], event["end_time"] * self.sim_data["step_len"], color="red", alpha=0.2)

            ax.axvline(event["start_time"] * self.sim_data["step_len"], color="red", alpha=0.4, linestyle='--', label=event["id"]+" start")
            ax.axvline(event["end_time"] * self.sim_data["step_len"], color="red", alpha=0.4, linestyle='--', label=event["id"]+" end")

            ax.text(event["start_time"] * self.sim_data["step_len"] + ((event["end_time"] - event["start_time"]) * self.sim_data["step_len"]/2), y_lim[1] * 0.9, event["id"], horizontalalignment='center', color="red")

def get_cumulative_arr(arr: list, start: int=0) -> list:
    arr = [0] + arr
    for i in range(start + 1, len(arr)):
        arr[i] += arr[i - 1]
    return arr[1:]

if __name__ == "__main__":
    plotter = Plotter("rm_demo.json")
    plotter.plot_meter_queue_length("crooswijk_meter")