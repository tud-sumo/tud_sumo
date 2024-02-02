import json, math, csv, matplotlib.pyplot as plt, numpy as np
from mpl_toolkits.axes_grid1 import make_axes_locatable
import os.path
from copy import deepcopy
from simulation import Simulation
from utils import *

default_labels = {"no_vehicles": "No. of Vehicles", "tts": "Total Time Spent (s)", "delay": "Delay (s)",
                  "veh_counts": "No. of Vehicles", "occupancies": "Occupancy (%)", "densities": "Density unit"}

default_titles = {"no_vehicles": "Number of Vehicles", "tts": "Total Time Spent", "delay": "Delay",
                  "veh_counts": "Number of Vehicles", "occupancies": "Vehicle Occupancies", "densities": "Vehicle Density",
                  "speeds": "Average Speed", "limits": "Speed Limit"}

class Plotter:
    def __init__(self, simulation: Simulation|str, sim_label: str|None=None, sim_time_units: str="s", fig_save_loc: str="", overwrite_figs: bool=True) -> None:
        """
        :param simulation:     Either simulation object, sim_data dict or sim_data filepath
        :param sim_label:      Simulation or scenario label added to the beginning of all plot titles
        :param fig_save_loc:   Figure filepath when saving (defaults to current file)
        :param overwrite_figs: Bool denoting whether to allow overwriting of saved figures with the same name
        """

        self.simulation = None
        if isinstance(simulation, Simulation):
            self.simulation = simulation
            self.sim_data = simulation.all_data
            self.units = simulation.units.name
            scenario_name = simulation.scenario_name

        elif isinstance(simulation, str):

            if os.path.exists(simulation):
                with open(simulation, "r") as fp:
                    self.sim_data = json.load(fp)
                    self.units = self.sim_data["units"]
                    scenario_name = self.sim_data["scenario_name"]
            else: raise FileNotFoundError("Plotter.init(): Simulation file '{0}' not found.".format(simulation))

        elif isinstance(simulation, dict): self.sim_data, self.units, scenario_name = simulation, simulation["units"], simulation["scenario_name"]

        else: raise TypeError("Plotter.init(): Invalid simulation type (must be Simulation|str|dict, not '{0}').".format(type(simulation).__name__))

        if sim_label != None and sim_label.upper() == "SCENARIO": self.sim_label = scenario_name + ": "
        elif sim_label != None: self.sim_label = sim_label + ": "
        else: self.sim_label = ""
        
        avg_speed, speed, limit = "Avg. Speed ", "Vehicle Speed ", "Speed Limit "

        if self.units in ["IMPERIAL", "UK"]:
            avg_speed += "(mph)"
            speed += "(mph)"
            limit += "(mph)"
        elif self.units in ["METRIC"]:
            avg_speed += "(km/h)"
            speed += "(km/h)"
            limit += "(km/h)"
        else: raise ValueError("Plotter.init(): Invalid simulation units '{0}' (must be 'METRIC'|'IMPERIAL'|'UK').".format(self.units.upper()))

        if sim_time_units.lower() in ["steps", "s", "m", "hr"]:
            self.sim_time_units = sim_time_units.lower()
        else: raise ValueError("Plotter.init(): Invalid simulation time unit '{0}' (must be 'steps'|'s'|'hr').".format(sim_time_units))
        
        default_labels["sim_time"] = "Simulation Time ({0})".format(self.sim_time_units)
        default_labels["speeds"] = avg_speed
        default_labels["speed"] = speed
        default_labels["limits"] = limit

        self.fig_save_loc, self.overwrite_figs = fig_save_loc, overwrite_figs
        if self.fig_save_loc != "":
            if not self.fig_save_loc.endswith('/'): self.fig_save_loc += "/"
            if not os.path.exists(self.fig_save_loc):
                raise FileNotFoundError("Plotter.init(): File path '{0}' does not exist.".format(self.fig_save_loc))

    def __name__(self):
        return "Plotter"
    
    def display_figure(self, filename: str|None=None) -> None:

        if filename is None: plt.show()
        else:
            
            if not filename.endswith(".png") and not filename.endswith('.jpg'):
                filename += ".png"

            fp = self.fig_save_loc + filename
            if os.path.exists(fp) and not self.overwrite_figs:
                raise FileExistsError("Plotter.display_figure() File '{0}' already exists.".format(fp))
            
            plt.savefig(fp, dpi=600)
        
    def plot_junc_flows(self, junc_id: str, vtypes: list|tuple|None=None, plot_n_vehicles: bool=False, plot_all: bool=True, save_fig: str|None=None) -> None:
        """
        Plot junction flow, either as inflow & outflow or number of vehicles at the intersection.
        :param junc_id:  Junction ID
        :param vtypes:   vType flows
        :param plot_n_vehicles: If true, plot number of vehicles in the intersection, else inflow and outflow
        :param plot_all: If true, plot total values as well as vType data
        :param save_fig: Output image filename, will show image if not given
        """

        if self.simulation != None:

            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

            if junc_id in self.simulation.tracked_juncs.keys(): tl = self.simulation.tracked_juncs[junc_id]
            else: raise KeyError("Plotter.plot_junc_flows(): Junction '{0}' not found in tracked junctions.".format(junc_id))

            if tl.track_flow: junc_flows = deepcopy(tl.get_curr_data()["flows"])
            else: raise ValueError("Plotter.plot_junc_flows(): No traffic light at junction '{0}'.".format(junc_id))

            start_time, end_time = tl.init_time, tl.curr_time
            sim_step_len = self.simulation.step_length

        elif "junctions" in self.sim_data["data"].keys() and junc_id in self.sim_data["data"]["junctions"].keys():
            if "flows" in self.sim_data["data"]["junctions"][junc_id].keys():
                junc_flows = deepcopy(self.sim_data["data"]["junctions"][junc_id]["flows"])
                start_time, end_time = self.sim_data["data"]["junctions"][junc_id]["init_time"], self.sim_data["data"]["junctions"][junc_id]["curr_time"]
                sim_step_len = self.sim_data["step_len"]

            else: raise ValueError("Plotter.plot_junc_flows(): Junction '{0}' does not track flows (no detectors).".format(junc_id))
        else: raise KeyError("Plotter.plot_junc_flows(): Junction '{0}' not found in tracked junctions.".format(junc_id))

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
        plt.xlabel(default_labels["sim_time"])
        plt.tight_layout()
        ax.legend(title="Types", fontsize="small", shadow=True)

        self.display_figure(save_fig)
    
    def plot_tl_colours(self, tl_id: str, plt_movements: list|tuple|None=None, time_range: list|tuple|None=None, plot_percent: bool=False, save_fig: str|None=None) -> None:
        """
        Plot traffic light sequence, as colours or green/red/yellow durations as a percent of time.
        :param tl_id:         Traffic light ID
        :param plt_movements: List of movements to plot by index (defaults to all)
        :param time_range:    Time range to plot
        :param plot_percent:  Denotes whether to plot colours as percent of time
        :param save_fig:      Output image filename, will show image if not given
        """

        plt_colour = {"G": "green", "Y": "yellow", "R": "red"}

        if self.simulation != None:

            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

            if tl_id in self.simulation.tracked_juncs.keys(): tl = self.simulation.tracked_juncs[tl_id]
            else: raise KeyError("Plotter.plot_tl_colours(): Junction '{0}' not found in tracked junctions.".format(tl_id))

            if tl.has_tl: tl_durs = deepcopy(tl.durations)
            else: raise ValueError("Plotter.plot_tl_colours(): No traffic light at junction '{0}'.".format(tl_id))

            m_len = tl.m_len
            init_time = tl.init_time

        elif "junctions" in self.sim_data["data"].keys() and tl_id in self.sim_data["data"]["junctions"].keys():
            if "tl" in self.sim_data["data"]["junctions"][tl_id].keys():
                tl_durs = deepcopy(self.sim_data["data"]["junctions"][tl_id]["tl"]["m_phases"])
                m_len, init_time = self.sim_data["data"]["junctions"][tl_id]["tl"]["m_len"], self.sim_data["data"]["junctions"][tl_id]["init_time"]

            else: raise ValueError("Plotter.plot_tl_colours(): No traffic light at junction '{0}'.".format(tl_id))
        else: raise KeyError("Plotter.plot_tl_colours(): Junction '{0}' not found in tracked junctions.".format(tl_id))

        if plt_movements != None:
            m_mask = plt_movements
            m_mask.sort()
            for idx in m_mask:
                if idx >= m_len or idx < 0: raise ValueError("Plotter.plot_tl_colours(): Invalid movement index '{0}' (must be 0 <= idx <= {1})".format(idx, m_len - 1))
            for i in reversed(range(m_len)):
                if i not in m_mask: tl_durs.pop(i)

            m_len = len(m_mask)

        xlim = self.sim_data["end"] * self.sim_data["step_len"]
        if isinstance(time_range, (list, tuple)) and time_range != None:
            if len(time_range) != 2: raise ValueError("Plotter.plot_tl_colours(): Invalid time range (must have length 2, not {0}).".format(len(time_range)))
            elif time_range[0] >= time_range[1]: raise ValueError("Plotter.plot_tl_colours(): Invalid time range (start_time ({0}) >= end_time ({1})).".format(start_time, end_time))
            else:
                clipped_tl_durs = []
                start_time, end_time = time_range[0], time_range[1]
                xlim = end_time * self.sim_data["step_len"]
                for m in tl_durs:
                    phase_times, phase_colours = [c_dur for (_, c_dur) in m], [colour for (colour, _) in m]
                    cum_phase_times = list(np.cumsum(phase_times))
                    if start_time < 0 or end_time > cum_phase_times[-1]:
                        raise ValueError("Plotter.plot_tl_colours(): Invalid time range (values [{0}-{1}] must be in range [0-{2}]).".format(start_time, end_time, cum_phase_times[-1]))

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
            plt.xlabel(default_labels["sim_time"])
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
        
        self.display_figure(save_fig)

    def plot_rm_rate(self, rm_id: str, ax=None, yax_labels: bool=True, xax_labels: bool=True, time_range: list|tuple|None=None, show_legend: bool=True, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot ramp metering rate.
        :param rm_id:       Ramp meter junction ID
        :param ax:          Matplotlib axis, used when creating subplots
        :param yax_labels:  Bool denoting whether to include y-axis labels (for subplots)
        :param xax_labels:  Bool denoting whether to include x-axis labels (for subplots)
        :param time_range:  Plotting time range (in plotter class units)
        :param show_legend: Bool denoting whether to show figure legend
        :param show_events: Bool denoting whether to plot events on all axes
        :param fig_title:   If given, will overwrite default title
        :param save_fig:    Output image filename, will show image if not givenr
        """

        if self.simulation != None:

            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

            if rm_id in self.simulation.tracked_juncs.keys(): tl = self.simulation.tracked_juncs[rm_id]
            else: raise KeyError("Plotter.plot_rm_rate(): Junction '{0}' not found in tracked junctions.".format(rm_id))

            if tl.is_meter:
                rates = tl.metering_rates
                times = tl.rate_times
                min_r, max_r = tl.min_rate, tl.max_rate
            else: raise ValueError("Plotter.plot_rm_rate(): Junction '{0}' is not tracked as a meter.".format(rm_id))

        elif "junctions" in self.sim_data["data"].keys() and rm_id in self.sim_data["data"]["junctions"].keys():
            if "meter" in self.sim_data["data"]["junctions"][rm_id].keys():
                rates = self.sim_data["data"]["junctions"][rm_id]["meter"]["metering_rates"]
                times = self.sim_data["data"]["junctions"][rm_id]["meter"]["rate_times"]
                min_r, max_r = self.sim_data["data"]["junctions"][rm_id]["meter"]["min_rate"], self.sim_data["data"]["junctions"][rm_id]["meter"]["max_rate"]

            else: raise ValueError("Plotter.plot_rm_rate(): Junction '{0}' is not tracked as a meter.".format(rm_id))
        else: raise KeyError("Plotter.plot_rm_rate(): Junction '{0}' not found in tracked junctions.".format(rm_id))

        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        start = convert_time_units(start, self.sim_time_units, step)
        end = convert_time_units(end, self.sim_time_units, step)
        times = convert_time_units(times, self.sim_time_units, step)

        is_subplot = ax != None
        if not is_subplot: fig, ax = plt.subplots(1, 1)
        
        colour = 'tab:blue'
        prev, label = None, "Metering Rate"
        for idx, val in enumerate(rates):
            if prev != None:
                
                line_start, line_end = times[int(idx - 1)], times[idx]
                if time_range != None:
                    if line_start >= time_range[1] or line_end <= time_range[0]: continue
                    else: line_start, line_end = max(line_start, time_range[0]), min(line_end, time_range[1])

                ax.plot([line_start, line_end], [prev, prev], label=label, color=colour, linewidth=1.5, zorder=3)
                
                # Vertical Line
                if time_range == None or (times[idx] > time_range[0] and times[idx] < time_range[1]):
                    ax.plot([times[idx], times[idx]], [prev, val], color=colour, linewidth=1.5, zorder=3)
                label = None

            prev = val

        if label != None: ax.plot([-1, -2], [-1, -2], label=label, color=colour, linewidth=1.5, zorder=3)

        last_line = [times[-1], end]
        if time_range != None:
            if last_line[0] >= time_range[1] or last_line[1] <= time_range[0]: last_line = None
            else: last_line[0], last_line[1] = max(last_line[0], time_range[0]), min(last_line[1], time_range[1])
        
        if last_line != None: ax.plot(last_line, [rates[-1], rates[-1]], color=colour, linewidth=1.5, zorder=3)

        xlim = [start, end]
        if time_range != None:
            xlim[0], xlim[1] = max(xlim[0], time_range[0]), min(xlim[1], time_range[1])
        ax.set_xlim(xlim)
        ax.set_ylim([0, get_axis_lim(max_r)])
        ax.axhline(max_r, label="Min/Max Rate", color="green", linestyle="--", alpha=0.5, zorder=1)
        ax.axhline(min_r, color="green", linestyle="--", alpha=0.5, zorder=2)
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)
        if yax_labels: ax.set_ylabel("Metering Rate (veh/hr)")
        if xax_labels: ax.set_xlabel(default_labels["sim_time"])
        fig_title = "{0}'{1}' Metering Rate".format(self.sim_label, rm_id) if not isinstance(fig_title, str) else fig_title
        if fig_title != "": ax.set_title(fig_title, pad=20)

        if show_legend:
            box = ax.get_position()
            if not is_subplot:
                ax.set_position([box.x0, box.y0 + box.height * 0.08,
                                box.width, box.height * 0.92])
                ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13),
                        fancybox=True, ncol=2)
            else:
                ax.set_position([box.x0, box.y0 + box.height * 0.02,
                                box.width, box.height * 0.80])
                ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                        fancybox=True, ncol=2)
            
        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)

        if not is_subplot:
            self.display_figure(save_fig)


    def plot_rm_rate_detector_data(self, rm_ids: str|list|tuple, all_detector_ids: list|tuple, data_keys: list|tuple, time_range: list|tuple|None=None, aggregate_data: int=10, data_titles: list|tuple|None=None, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot ramp metering rate next to detector data.
        :param rm_ids:         Ramp meter junction ID or list of IDs
        :param detector_ids:   List of detector IDs or nested list for multiple meters
        :param data_keys:      Plotting data keys ["speeds", "veh_counts", "occupancies"]
        :param time_range:     Plotting time range (in plotter class units)
        :param aggregate_data: Averaging interval in steps (defaults to 10)
        :param data_titles:    List of axes titles, if given must have same length as data_keys
        :param show_events:    Bool denoting whether to plot events on all axes
        :param fig_title:      If given, will overwrite default title
        :param save_fig:       Output image filename, will show image if not given
        """
        
        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        if not isinstance(rm_ids, (list, tuple)): rm_ids = [rm_ids]
        if not isinstance(all_detector_ids[0], (list, tuple)): all_detector_ids = [all_detector_ids]

        if len(rm_ids) != len(all_detector_ids):
            raise ValueError("Plotter.plot_rm_rate_detector_data(): Number of rm_ids '{0}' and all_detector_ids groups '{1}' do not match.".format(len(rm_ids), len(all_detector_ids)))

        if isinstance(data_titles, (list, tuple)) and len(data_titles) != len(data_keys):
            raise KeyError("Plotter.plot_rm_rate_detector_data(): Length of data_keys and data_titles do not match.")

        fig_dimensions = 4 if len(rm_ids) == 1 else 3
        fig, all_axes = plt.subplots(len(rm_ids), 1+len(data_keys), figsize=((1+len(data_keys))*fig_dimensions, fig_dimensions*len(rm_ids)))
        
        if len(rm_ids) == 1: all_axes = [all_axes]
        else:
            new_axes = []
            new_row = []
            for col_idx in range(1+len(data_keys)):
                for rm_idx in range(len(rm_ids)):
                    new_row.append(all_axes[rm_idx][col_idx])
                new_axes.append(new_row)
                new_row = []
            all_axes = new_axes


        for rm_idx, (rm_id, detector_ids, axes) in enumerate(zip(rm_ids, all_detector_ids, all_axes)):
            self.plot_rm_rate(rm_id, axes[0],
                                    yax_labels=rm_idx==0, xax_labels=len(rm_ids)==1,
                                    show_legend=len(rm_ids)==1,
                                    time_range=time_range, show_events=show_events,
                                    fig_title="Metering Rate" if len(rm_ids) == 1 else rm_id)
            line_colours = plt.rcParams['axes.prop_cycle'].by_key()['color']

            for idx, (data_key, ax) in enumerate(zip(data_keys, axes[1:])):

                all_detector_data = []
                for det_id in detector_ids:
                    if "detector" in self.sim_data["data"].keys():
                        if det_id in self.sim_data["data"]["detector"].keys():
                            if data_key in self.sim_data["data"]["detector"][det_id].keys():
                                det_data = self.sim_data["data"]["detector"][det_id][data_key]
                                all_detector_data.append(det_data)
                            else: raise KeyError("Plotter.plot_rm_rate_detector_data(): Unrecognised dataset key '{0}'.".format(data_key))
                        else: raise KeyError("Plotter.plot_rm_rate_detector_data(): Unrecognised detector ID '{0}'.".format(det_id))
                    else: raise KeyError("Plotter.plot_rm_rate_detector_data(): No detector data to plot.")

                if len(set([len(data) for data in all_detector_data])) == 1:
                    n_steps = len(all_detector_data[0])
                else: raise AssertionError("Plotter.plot_rm_rate_detector_data(): Mismatching detector data lengths.")

                avg_data, steps, curr_step = [], [], start
                if time_range == None: time_range = [-math.inf, math.inf]
                
                while curr_step < end:
                    if curr_step < time_range[0]: 
                        curr_step += 1
                        continue
                    elif curr_step > time_range[1]: break
                    
                    step_data = [all_detector_data[det_idx][curr_step - start] for det_idx in range(len(detector_ids))]
                    step_data = [val for val in step_data if val != -1]

                    if len(step_data) > 0: avg_data.append(sum(step_data) / len(step_data))
                    else: avg_data.append(-1)
                    
                    steps.append(curr_step)
                    curr_step += 1

                if isinstance(aggregate_data, (int, float)):
                    avg_data, steps = get_aggregated_data(avg_data, steps, int(aggregate_data / step))
                
                steps = convert_time_units(steps, self.sim_time_units, step)
                ax.plot(steps, avg_data, color=line_colours[1+idx])

                xlim = [convert_time_units(start, self.sim_time_units, step), convert_time_units(end, self.sim_time_units, step)]
                if time_range != None:
                    xlim[0], xlim[1] = max(xlim[0], time_range[0]), min(xlim[1], time_range[1])

                ax.set_xlim(xlim)
                ax.set_ylim([0, get_axis_lim(avg_data)])
                ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)
                if rm_idx == 0: ax.set_ylabel(default_labels[data_key])
                if len(rm_ids) == 1 or data_key == data_keys[-1]: ax.set_xlabel(default_labels["sim_time"])
                
                if len(rm_ids) == 1:
                    if data_titles == None: ax.set_title(default_titles[data_key], pad=20)
                    else: ax.set_title(data_titles[idx], pad=20)

                if "events" in self.sim_data["data"].keys() and show_events:
                    if "completed" in self.sim_data["data"]["events"]:
                        self.plot_event(ax)

        if len(rm_ids) == 1:
            fig_title = "{0}'{1}' Data".format(self.sim_label, rm_id) if not isinstance(fig_title, str) else fig_title
        else: fig_title = "{0}Ramp Metering & Detector Data".format(self.sim_label, rm_id) if not isinstance(fig_title, str) else fig_title
        if fig_title != "": fig.suptitle(fig_title, fontweight='bold')
        fig.tight_layout()
        self.display_figure(save_fig)

    def plot_rm_queuing(self, rm_id: str, ax=None, yax_labels: bool|list|tuple=True, xax_labels: bool=True, plot_delay: bool=True, time_range: list|tuple|None=None, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot ramp metering rate.
        :param rm_id:       Ramp meter junction ID
        :param ax:          Matplotlib axis, used when creating subplots
        :param yax_labels:  Bool denoting whether to include y-axis labels (for subplots). Either single bool for both y-axis labels or list of two bools to set both y-axes (when plotting delay).
        :param xax_labels:  Bool denoting whether to include x-axis labels (for subplots)
        :param plot_delay:  Bool denoting whether to plot queue delay. This will be done on the same plot with a separate y-axis.
        :param time_range:  Plotting time range (in plotter class units)
        :param show_events: Bool denoting whether to plot events on all axes
        :param fig_title:   If given, will overwrite default title
        :param save_fig:    Output image filename, will show image if not givenr
        """

        if self.simulation != None:

            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

            if rm_id in self.simulation.tracked_juncs.keys(): tl = self.simulation.tracked_juncs[rm_id]
            else: raise KeyError("Plotter.plot_rm_queuing(): Junction '{0}' not found in tracked junctions.".format(rm_id))

            if tl.is_meter: 
                if tl.measure_queues:
                    queue_lengths = tl.queue_lengths
                    queue_delays = tl.queue_delays
                else: raise ValueError("Plotter.plot_rm_queuing(): Meter '{0}' does not track queue lengths (no queue detector).".format(rm_id))
            else: raise ValueError("Plotter.plot_rm_queuing(): Junction '{0}' is not tracked as a meter.".format(rm_id))

        elif "junctions" in self.sim_data["data"].keys() and rm_id in self.sim_data["data"]["junctions"].keys():
            if "meter" in self.sim_data["data"]["junctions"][rm_id].keys():
                if "queue_lengths" in self.sim_data["data"]["junctions"][rm_id]["meter"].keys():
                    queue_lengths = self.sim_data["data"]["junctions"][rm_id]["meter"]["queue_lengths"]
                    queue_delays = self.sim_data["data"]["junctions"][rm_id]["meter"]["queue_delays"]
                else: raise ValueError("Plotter.plot_rm_queuing(): Meter '{0}' has not tracked queue lengths (no queue detector).".format(rm_id))
            else: raise ValueError("Plotter.plot_rm_queuing(): Junction '{0}' has not been tracked as a meter.".format(rm_id))
        else: raise KeyError("Plotter.plot_rm_queuing(): Junction '{0}' not found in tracked junctions.".format(rm_id))

        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        is_subplot = ax != None
        if not is_subplot: fig, ax1 = plt.subplots(1, 1)
        else: ax1 = ax

        colour = 'tab:blue'
        all_data_time_vals = convert_time_units([x for x in range(start, end)], self.sim_time_units, step)
        data_time_vals, queue_lengths = limit_vals_by_range(all_data_time_vals, queue_lengths, time_range)
        ax1.plot(data_time_vals, queue_lengths, linewidth=1.5, zorder=3, color=colour)
        if xax_labels: ax1.set_xlabel(default_labels["sim_time"])
        if (isinstance(yax_labels, bool) and yax_labels) or (isinstance(yax_labels, (list, tuple)) and len(yax_labels) == 2 and yax_labels[0]):
            ax1.set_ylabel("No. of On-ramp Vehicles")
        else: TypeError("Plotter.plot_rm_queuing(): Invalid yax_label, must be bool or list of 2 bools denoting each axis.")

        if time_range == None: time_range = [-math.inf, math.inf]
        ax1.set_xlim([max(time_range[0], data_time_vals[0]), min(time_range[1], data_time_vals[-1])])
        ax1.set_ylim([0, get_axis_lim(queue_lengths)])

        ax1.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)
        if not is_subplot or fig_title != None:
            if not isinstance(fig_title, str):
                default_title = "{0}'{1}' Queue Lengths".format(self.sim_label, rm_id)
                if plot_delay: default_title += " & Cumulative Delay"
                fig_title = default_title
            ax1.set_title(fig_title, pad=20)

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax1)

        if plot_delay:
            ax1.tick_params(axis='y', labelcolor=colour)
            if (isinstance(yax_labels, bool) and yax_labels) or (isinstance(yax_labels, (list, tuple)) and len(yax_labels) == 2 and yax_labels[0]):
                ax1.set_ylabel("No. of On-ramp Vehicles", color=colour)
        
            colour = 'tab:red'
            data_time_vals, queue_delays = limit_vals_by_range(all_data_time_vals, queue_delays, time_range)
            queue_delays = get_cumulative_arr(queue_delays)
            ax2 = ax1.twinx()

            ax2.plot(data_time_vals, queue_delays, linewidth=1.5, zorder=3, color=colour)
            ax2.tick_params(axis='y', labelcolor=colour)
            if (isinstance(yax_labels, bool) and yax_labels) or (isinstance(yax_labels, (list, tuple)) and len(yax_labels) == 2 and yax_labels[1]):
                ax2.set_ylabel("Cumulative Delay (s)", color=colour)
            else: TypeError("Plotter.plot_rm_queuing(): Invalid yax_label, must be bool or list of 2 bools denoting each axis.")
            ax2.set_ylim([0, get_axis_lim(queue_delays)])

        if not is_subplot:
            fig.tight_layout()
            self.display_figure(save_fig)

    def plot_rm_rate_queuing(self, rm_ids: str|list|tuple, plot_queuing: bool=True, time_range: list|tuple|None=None, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot meter queue length and delay.
        :param rm_ids:       Ramp meter junction ID or list of IDs
        :param plot_queuing: Bool denoting whether to plot queue lengths and delay (set False to only plot metering rate)
        :param time_range:   Plotting time range (in plotter class units)
        :param show_events:  Bool denoting whether to plot when events occur
        :param fig_title:    If given, will overwrite default title
        :param save_fig:     Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        if not isinstance(rm_ids, (list, tuple)): rm_ids = [rm_ids]
        
        fig_dimensions = 4
        if len(rm_ids) == 1:
            if plot_queuing:
                fig, (ax, ax2) = plt.subplots(1, 2, figsize=(fig_dimensions*2, fig_dimensions))
                self.plot_rm_queuing(rm_ids[0], ax2, True, True, True, time_range, show_events, fig_title="Queue Lengths & Delays")
            else: fig, ax = plt.subplots(1, 1, figsize=(fig_dimensions*2, fig_dimensions))
            self.plot_rm_rate(rm_ids[0], ax,
                                    yax_labels=True, xax_labels=True,
                                    time_range=time_range,
                                    show_legend=True,
                                    show_events=show_events,
                                    fig_title="Metering Rate")
        
        else:
            nrows, ncols = 2 if plot_queuing else 1, len(rm_ids)
            fig, axes = plt.subplots(nrows, ncols, figsize=(ncols*fig_dimensions*1.2, nrows*fig_dimensions))

            for idx, rm_id in enumerate(rm_ids):
                ax = axes[0][idx] if plot_queuing else axes[idx]
                self.plot_rm_rate(rm_id, ax,
                                        yax_labels=idx==0,
                                        xax_labels=not plot_queuing,
                                        time_range=time_range,
                                        show_legend=False,
                                        show_events=show_events,
                                        fig_title=rm_id)
                
                if plot_queuing:
                    self.plot_rm_queuing(rm_id, axes[1][idx], (idx==0, idx==len(rm_ids)-1), True, True, time_range, show_events, "")

        def_title = "Ramp Metering Rates"
        if plot_queuing: def_title += " & Queuing Data"
        fig_title = self.sim_label+def_title if not isinstance(fig_title, str) else fig_title
        if fig_title != "": fig.suptitle(fig_title, fontweight='bold')

        fig.tight_layout()
        self.display_figure(save_fig)

    def plot_vehicle_detector_data(self, dataset: str|list|tuple, save_fig: str|None=None) -> None:
        """
        Plots all collected vehicle or detector data.
        :param dataset:  Dataset key (either "vehicle" or ["detector", detector_id])
        :param save_fig: Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        if isinstance(dataset, str): # Sim data
            if dataset in self.sim_data["data"].keys(): data = self.sim_data["data"][dataset]
            else: raise KeyError("Plotter.plot_vehicle_detector_data(): Unrecognised dataset key '{0}'.".format(dataset))
            title = self.sim_label + dataset.title() + " Data"
        elif isinstance(dataset, (list, tuple)): # Detector data
            data = self.sim_data["data"]
            if "detector" not in data.keys():
                raise KeyError("Plotter.plot_vehicle_detector_data(): No detector data to plot.")
            for key in dataset:
                if key in data.keys(): data = data[key]
                else: raise KeyError("Plotter.plot_vehicle_detector_data(): Unrecognised dataset key '{0}'.".format(dataset[-1]))
            title = self.sim_label + ': '.join(dataset)

        else: raise TypeError("Plotter.plot_vehicle_detector_data(): Invalid dataset key type (must be [int|str], not '{0}').".format(type(dataset).__name__))

        plot_data = {key: data[key] for key in data if key in default_labels and len(data[key]) != 0}
        fig, axes = plt.subplots(len(plot_data))
        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        for idx, (ax, (data_key, data_vals)) in enumerate(zip(axes, plot_data.items())):
            if not isinstance(data_vals, list): continue
            ax.plot([x * step for x in range(start, end)], data_vals, zorder=3)
            ax.set_title(data_key)
            if data_key in default_labels.keys(): ax.set_ylabel(default_labels[data_key])
            if idx < len(axes) - 1:
                if idx == 0: ax.set_title(title)
                ax.tick_params('x', labelbottom=False)
            else: ax.set_xlabel('Simulation Time (s)')
            ax.set_xlim([start * step, (end - 1) * step])
            ax.set_ylim([0, get_axis_lim(data_vals)])
            ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

        fig.tight_layout()

        self.display_figure(save_fig)

    def plot_n_vehicles(self, cumulative: bool=True, show_events: bool=True, save_fig: str|None=None) -> None:
        """
        Plot the number of vehicles in the system.
        :param cumulative: Bool denoting whether values are displayed cumulatively
        :param show_events: Bool denoting whether to plot when events occur
        :param save_fig: Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

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
        ax.set_ylim([0, get_axis_lim(data_vals)])
        ax.set_xlabel(default_labels["sim_time"])
        ax.set_ylabel(xlabel)
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)
        
        fig.tight_layout()

        self.display_figure(save_fig)

    def plot_cumulative_curve(self, inflow_detectors: list|tuple|None=None, outflow_detectors: list|tuple|None=None, outflow_offset: int|float=0, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot inflow and outflow cumulative curves, either system-wide or using inflow/outflow detectors (if given).
        :param inflow_detectors: List of inflow detectors
        :param outflow_detectors: List of outflow detectors
        :param outflow_offset: Offset for outflow values if not starting at t=0
        :param show_events: Bool denoting whether to plot when events occur
        :param fig_title: If given, will overwrite default title
        :param save_fig: Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        inflows, outflows = [], []

        if inflow_detectors == None and outflow_detectors == None:

            if "all_vehicles" not in self.sim_data["data"].keys():
                raise KeyError("Plotter.plot_cumulative_curve(): Plot requires 'individual_vehicle_data=True' during the simulation to plot system cumulative curves.")
            else: all_vehicle_data = self.sim_data["data"]["all_vehicles"]
            
            prev_vehicles = set([])
            
            for step, vehicles in enumerate(all_vehicle_data):
                curr_vehicles = set(vehicles.keys())
                inflows.append(len(curr_vehicles - prev_vehicles))
                outflows.append(len(prev_vehicles - curr_vehicles))
                prev_vehicles = curr_vehicles
        else:
            if inflow_detectors == None or outflow_detectors == None:
                raise TypeError("Plotter.plot_cumulative_curve(): If using detectors, both inflow and outflow detectors are required.")
            
            if "detector" not in self.sim_data["data"].keys():
                raise KeyError("Plotter.plot_vehicle_detector_data(): No detector data to plot.")
            
            detector_data = self.sim_data["data"]["detector"]
            if not hasattr(inflow_detectors, "__iter__"): inflow_detectors = [inflow_detectors]
            if not hasattr(outflow_detectors, "__iter__"): outflow_detectors = [outflow_detectors]

            if len(set(inflow_detectors + outflow_detectors) - set(detector_data.keys())) != 0:
                raise KeyError("Plotter.plot_cumulative_curve(): Detectors '{0}' could not be found.".format(set(inflow_detectors + outflow_detectors) - set(detector_data.keys())))

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
        fig_title = "{0}Cumulative Arrival-Departure Curve".format(self.sim_label) if not isinstance(fig_title, str) else fig_title
        ax.set_title(fig_title, pad=20)
        
        ax.plot(x_vals, inflows, label="Inflow", zorder=3)
        ax.plot(x_vals, outflows, label="Outflow", zorder=4)
        ax.set_xlim([start * step, (end - 1) * step])
        ax.set_ylim([0, get_axis_lim(inflows)])
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)
        ax.set_xlabel(default_labels["sim_time"])
        ax.set_ylabel("Cumulative No. of Vehicles")
        ax.legend(loc='lower right', shadow=True)

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)

        fig.tight_layout()

        self.display_figure(save_fig)

    def plot_vsl_data(self, vsl_id: str, avg_geomtry_speeds: bool=False, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot VSL settings and average vehicle speeds on affected edges.
        :param vsl_id: VSL controller ID
        :param avg_geometry_speeds: Bool denoting whether to plot average edge speed, or individual edge data
        :param show_events: Bool denoting whether to plot when events occur
        :param fig_title: If given, will overwrite default title
        :param save_fig: Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        if "controllers" not in self.sim_data["data"].keys():
            raise KeyError("Plotter.plot_vsl_data(): No controllers used during simulation.")
        elif vsl_id not in self.sim_data["data"]["controllers"].keys():
            raise KeyError("Plotter.plot_vsl_data(): Controller ID '{0}' not found.".format(vsl_id))
        
        vsl_data = self.sim_data["data"]["controllers"][vsl_id]
        if vsl_data["type"] != "VSL":
            raise KeyError("Plotter.plot_vsl_data(): Controller '{0}' is not a VSL controller.".format(vsl_id))
        
        start, end, step = vsl_data["init_time"], vsl_data["curr_time"], self.sim_data["step_len"]
        
        colour = 'green'
        limits, times = vsl_data["speed_limits"], vsl_data["times"]

        if len(limits) == 0:
            raise ValueError("Plotter.plot_vsl_data(): VSL controller '{0}' has no data, likely was not activated.".format(vsl_id))

        fig, ax = plt.subplots(1, 1)

        prev = None
        activated = False
        active_times, activated_time = [], None
        label = "Speed Limit"
        linewidth = 1.5 if avg_geomtry_speeds else 2
        for idx, val in enumerate(limits):
            if prev == None: prev = val
            else:
                if prev != -1 and val != -1:
                    ax.plot([times[idx] * step, times[idx] * step], [prev, val], color=colour, linewidth=linewidth, label=label, zorder=3)
                    label = None
                
            if val != -1:
                if not activated:
                    activated = True
                    if activated_time == None: activated_time = times[idx] * step

                if idx == len(limits) - 1: line_x_lim = end * step
                else: line_x_lim = times[idx + 1] * step
                ax.plot([times[idx] * step, line_x_lim], [val, val], color=colour, linewidth=linewidth, zorder=3)
            else:
                active_times.append([activated_time, times[idx] * step])
                activated, activated_time = False, None
            
            prev = val

        label = "VSL Activated"
        for ranges in active_times:
            for time in ranges:
                ax.axvline(time, color="grey", alpha=0.2, linestyle='--')
            ax.axvspan(ranges[0], ranges[1], color="grey", alpha=0.1, label=label)
            label = None
        
        edge_ids = list(vsl_data["geometry_data"].keys())
        edge_speeds = [vsl_data["geometry_data"][e_id]["avg_speeds"] for e_id in edge_ids]
        n_edges = len(edge_speeds)
        if avg_geomtry_speeds:
            avg_speeds = []
            for time_idx in range(len(edge_speeds[0])):
                all_pos_vals = [edge_speeds[edge_idx][time_idx] for edge_idx in range(n_edges) if edge_speeds[edge_idx][time_idx] != -1]
                if len(all_pos_vals) == 0: avg_speeds.append(-1)
                else: avg_speeds.append(sum(all_pos_vals) / len(all_pos_vals))

            edge_speeds = [avg_speeds]

        max_speed = max(limits)
        for edge_idx, edge in enumerate(edge_speeds):
            if avg_geomtry_speeds: label = "Avg. Edge Speed"
            else: label = "'{0}' Speed".format(edge_ids[edge_idx])
            prev_line = None
            x_vals, y_vals = [], []
            curr_time = start
            for speed_val in edge:
                max_speed = max(max_speed, speed_val)
                if speed_val == -1:
                    if len(x_vals) != 0:
                        if prev_line == None:
                            prev_line, label = ax.plot(x_vals, y_vals, label=label, linewidth=1), None
                        else: prev_line = ax.plot(x_vals, y_vals, color=prev_line[0].get_color(), label=label, linewidth=1)
                        x_vals, y_vals = [], []
                else:
                    x_vals.append(curr_time)
                    y_vals.append(speed_val)

                curr_time += step

            if len(x_vals) != 0 and len(y_vals) != 0:
                if prev_line == None: prev_line = ax.plot(x_vals, y_vals, label=label, linewidth=1)
                else: prev_line = ax.plot(x_vals, y_vals, color=prev_line[0].get_color(), label=label, linewidth=1)

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)

        y_lim = get_axis_lim(max_speed)
        ax.set_ylim(0, y_lim)
        ax.set_xlim(start * step, end * step)

        ax.set_xlabel(default_labels["sim_time"])
        ax.set_ylabel(default_labels["limits"])

        fig_title = "{0}'{1}' Speed Limit and Average Vehicle Speed".format(self.sim_label, vsl_id) if not isinstance(fig_title, str) else fig_title
        ax.set_title(fig_title, pad=20)
        
        box = ax.get_position()
        ax.set_position([box.x0, box.y0 + box.height * 0.02,
                        box.width, box.height * 0.98])

        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.14),
          fancybox=True, ncol=3)

        fig.tight_layout()

        self.display_figure(save_fig)

    def plot_rg_data(self, rg_id: str, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot how many vehicles are diverted by RG controller.
        :param rg_id: RG controller ID
        :param show_events: Bool denoting whether to plot when events occur
        :param fig_title: If given, will overwrite default title
        :param save_fig: Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        if "controllers" not in self.sim_data["data"].keys():
            raise KeyError("Plotter.plot_rg_data(): No controllers used during simulation.")
        elif rg_id not in self.sim_data["data"]["controllers"].keys():
            raise KeyError("Plotter.plot_rg_data(): Controller ID '{0}' not found.".format(rg_id))
        
        rg_data = self.sim_data["data"]["controllers"][rg_id]
        if rg_data["type"] != "RG":
            raise KeyError("Plotter.plot_rg_data(): Controller '{0}' is not a RG controller.".format(rg_id))
        
        start, end, step = rg_data["init_time"], rg_data["curr_time"], self.sim_data["step_len"]
        data_vals = get_cumulative_arr(rg_data["n_diverted"])

        if len(data_vals) == 0:
            raise ValueError("Plotter.plot_vsl_data(): RG controller '{0}' has no data, likely was not activated.".format(rg_id))
        
        fig, ax = plt.subplots(1, 1)

        ax.plot([x * step for x in range(start, end)], data_vals, zorder=3, label="Diverted Vehicles")
        ax.set_xlim([start * step, (end - 1) * step])
        y_lim = get_axis_lim(data_vals)
        ax.set_ylim([0, y_lim])
        ax.set_xlabel(default_labels["sim_time"])
        ax.set_ylabel("No. of Diverted Vehicles")
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

        active_times = rg_data["activation_times"]
        if isinstance(active_times[-1], (int, float)):
            active_times[-1] = (active_times[-1], end)

        label = "RG Activated"
        for ranges in active_times:
            for time in ranges:
                ax.axvline(time * step, color="grey", alpha=0.2, linestyle='--')
            ax.axvspan(ranges[0] * step, ranges[1] * step, color="grey", alpha=0.1, label=label)
            label = None

        fig_title = "{0}'{1}' Number of Diverted Vehicles".format(self.sim_label, rg_id) if not isinstance(fig_title, str) else fig_title
        ax.set_title(fig_title, pad=20)
        ax.legend()

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)

        fig.tight_layout()

        self.display_figure(save_fig)

    def plot_space_time_diagram(self, edge_ids: list|tuple, time_range: list|tuple|None=None, upstream_at_top: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot space time data from tracked edge data.
        :param edge_ids: Single tracked egde ID or list of IDs
        :param upstream_at_top: If true, upstream values are displayed at the top of the diagram
        :param show_events: Bool denoting whether to plot when events occur
        :param fig_title: If given, will overwrite default title
        :param save_fig: Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        if "edges" not in self.sim_data["data"].keys():
            raise KeyError("Plotter.plot_space_time_diagram(): No edges tracked during the simulation.")
        
        if not isinstance(edge_ids, (list, tuple)): edge_ids = [edge_ids]

        fig, ax = plt.subplots(1, 1)
        edge_offset = 0

        y_scale = 1
        x_vals, y_vals, speed_vals = [], [], []
        total_len = sum([self.sim_data["data"]["edges"][e_id]["length"] for e_id in edge_ids])

        if self.units in ["METRIC", "UK"]:
            if total_len >= 1000: y_scale, y_label = 0.001, "Distance (km)"
            else: y_label = "Distance (m)"
        elif self.units in ["IMPERIAL"]:
            y_label = "Distance (mi)"
        x_label = default_labels["sim_time"]

        if time_range == None: time_range = [-math.inf, math.inf]

        for e_id in edge_ids:
            if e_id not in self.sim_data["data"]["edges"].keys():
                raise KeyError("Plotter.plot_space_time_diagram(): Edge '{0}' not found in tracked edges.".format(e_id))
            else: e_data = self.sim_data["data"]["edges"][e_id]

            step_vehicles = e_data["step_vehicles"]
            
            edge_length = e_data["length"]
            start, step = e_data["init_time"], self.sim_data["step_len"]

            curr_step = start
            curr_time = convert_time_units(curr_step, self.sim_time_units, step)

            # Chane with new time unit settings! Also, currently will not plot anything
            # if there is no time_range, which does not make any sense
            for step_data in step_vehicles:
                if curr_time <= time_range[1] and curr_time >= time_range[0]:
                    for veh_data in step_data:

                        speed_vals.append(veh_data[1])
                        x_vals.append(curr_time)

                        y_val = (veh_data[0] * edge_length) + edge_offset
                        if not upstream_at_top: y_val = total_len - y_val
                        y_val *= y_scale
                        y_vals.append(y_val)
                elif curr_time > time_range[1]:
                    break

                curr_step += step
                curr_time = convert_time_units(curr_step, self.sim_time_units, step)

            edge_offset += edge_length

        if len(x_vals) == 0 or len(y_vals) == 0:
            if time_range == None:
                raise ValueError("Plotter.plot_space_time_diagram(): No data to plot (no vehicles recorded on edges).")
            else:
                raise ValueError("Plotter.plot_space_time_diagram(): No data to plot (no vehicles recorded during time frame '{0}-{1}{2}').".format(time_range[0], time_range[1], self.sim_time_units))
        
        points = ax.scatter(x_vals, y_vals, c=speed_vals, s=0.5, cmap='hot')

        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="5%", pad=0.1)
        plt.colorbar(points, cax=cax, label=default_labels["speed"])

        ax.set_xlim(min(x_vals), max(x_vals))
        ax.set_ylim(0, max(y_vals))
        ax.set_ylabel(y_label)
        ax.set_xlabel(x_label)

        if not isinstance(fig_title, str):
            if len(edge_ids) == 0: e_label = "Edge '{0}'".format(edge_ids[0])
            elif upstream_at_top: e_label = "Edges '{0}' - '{1}'".format(edge_ids[-1], edge_ids[0])
            else: e_label = "Edges '{0}' - '{1}'".format(edge_ids[0], edge_ids[-1])
            fig_title = "{0}{1} Vehicle Speeds and Positions".format(self.sim_label, e_label)

        ax.set_title(fig_title, pad=20)

        fig.tight_layout()

        self.display_figure(save_fig)

    def plot_event(self, ax) -> None:
        """
        Plot events from the simulation data on a given axes.
        :param ax: matplotlib.pyplot.axes object
        """

        _, y_lim = ax.get_xlim(), ax.get_ylim()
        for event in self.sim_data["data"]["events"]["completed"]:
            ax.axvspan(event["start_time"] * self.sim_data["step_len"], event["end_time"] * self.sim_data["step_len"], color="red", alpha=0.2)

            ax.axvline(event["start_time"] * self.sim_data["step_len"], color="red", alpha=0.4, linestyle='--')
            ax.axvline(event["end_time"] * self.sim_data["step_len"], color="red", alpha=0.4, linestyle='--')

            ax.text(event["start_time"] * self.sim_data["step_len"] + ((event["end_time"] - event["start_time"]) * self.sim_data["step_len"]/2), y_lim[1] * 0.9, event["id"], horizontalalignment='center', color="red")

def compare_vehicle_data(scenarios_data, data_key, labels = None, colours = None, linestyles = None, linewidths = None, cumulative=False, fig_title: str|None=None, save_fig: str|None=None) -> None:

    default_titles = {"no_vehicles": "Number of Vehicles", "c_no_vehicles": "Total Number of Vehicles",
                      "tts": "Total Time Spent", "c_tts": "Cumulative Time Spent",
                      "delay": "Vehicle Delay", "c_delay": "Total Vehicle Delay"}
    if labels == None: labels = [dat["scenario_name"] for dat in scenarios_data]

    fig, ax = plt.subplots(1, 1)

    min_x, max_x, max_y = math.inf, -math.inf, -math.inf
    for idx, (data, label) in enumerate(zip(scenarios_data, labels)):
        start, end, step = data["start"], data["end"], data["step_len"]
        plot_data = data["data"]["vehicle"][data_key]
        if cumulative: plot_data = get_cumulative_arr(plot_data)
        colour = None if colours == None else colours[idx]
        linestyle = 'solid' if linestyles == None else linestyles[idx]
        linewidth = 1 if linewidths == None else linewidths[idx]
        ax.plot([x * step for x in range(start, end)], plot_data, label=label, color=colour, linestyle=linestyle, linewidth=linewidth)

        min_x, max_x, max_y = min(min_x, start * step), max(max_x, end * step), max(max_y, max(plot_data))

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel(default_labels[data_key])
    ax.set_xlim([min_x, max_x])
    ax.set_ylim([0, get_axis_lim(max_y)])
    ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

    box = ax.get_position()
    ax.set_position([box.x0, box.y0 + box.height * 0.1, box.width, box.height * 0.90])

    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.14), fancybox=True, ncol=len(scenarios_data))
    if fig_title == None:
        if cumulative: fig_title = default_titles["c_"+data_key]
        else: fig_title = default_titles[data_key]
    if fig_title != "": ax.set_title(fig_title, pad=20)
    
    if save_fig is None: plt.show(block=True)
    else: plt.savefig(save_fig)

def plot_lppo_density(filename):
    with open(filename, 'r') as fp:
        reader = csv.reader(fp)

        data = {}
        for row in reader:
            rm_id = row[1]
            if rm_id not in data.keys():
                data[rm_id] = {'densities': [], 'rewards': [], 'steps': []}
            data[rm_id]['densities'].append(float(row[3]))
            data[rm_id]['rewards'].append(float(row[2]))
            data[rm_id]['steps'].append(int(row[0]))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
    
    min_t, max_t = math.inf, -math.inf
    for rm_id, rm_data in data.items():
        steps = convert_time_units(rm_data['steps'], "hr", 0.5)
        min_t, max_t = min(min(steps), min_t), max(max(steps), max_t)
        ax1.plot(steps, rm_data['densities'], label=rm_id, linewidth=1)
        ax2.plot(steps, rm_data['rewards'], label=rm_id, linewidth=1)

    ax1.set_title("Downstream Density")
    ax1.set_xlabel("Simulation Time (hr)")
    ax1.set_ylabel("Density (veh/km)")
    ax1.set_xlim([min_t, max_t])
    ax1.legend()
    
    ax2.set_title("Agent Reward")
    ax2.set_xlabel("Simulation Time (hr)")
    ax2.set_ylabel("Agent Reward")
    ax2.set_xlim([min_t, max_t])
    
    fig.suptitle("Local PPO Scenario: Downstream Density & Agent Reward", fontweight='bold')
    fig.tight_layout()
    plt.show()

def plot_cppo_density(filename):
    with open(filename, 'r') as fp:
        reader = csv.reader(fp)

        densities, rewards, steps = [], [], []
        for row in reader:
            densities.append(float(row[2]))
            rewards.append(float(row[1]))
            steps.append(int(row[0]))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
    
    steps = convert_time_units(steps, "hr", 0.5)
    min_t, max_t = min(min(steps), math.inf), max(max(steps), -math.inf)
    ax1.plot(steps, densities, linewidth=1)
    ax2.plot(steps, rewards, linewidth=1)

    ax1.set_title("Mainline Density")
    ax1.set_xlabel("Simulation Time (hr)")
    ax1.set_ylabel("Density (veh/km)")
    ax1.set_xlim([min_t, max_t])
    
    ax2.set_title("Agent Reward")
    ax2.set_xlabel("Simulation Time (hr)")
    ax2.set_ylabel("Agent Reward")
    ax2.set_xlim([min_t, max_t])
    
    fig.suptitle("Coordinated PPO Scenario: Downstream Density & Agent Reward", fontweight='bold')
    fig.tight_layout()
    plt.show()

if __name__ == "__main__":

    if False:

        for label, filename in zip(["No Control Scenario", "ALINEA", "Local PPO Scenario", "Coordinated PPO Scenario"], ["nc", "alinea", "lppo", "cppo"]):
            if filename != "cppo": continue
            print('loading '+filename)
            plotter = Plotter("ramp_metering/new_net_test_"+filename+".json", sim_label=label, sim_time_units="hr")
            print('loaded')
            plotter.plot_n_vehicles()
            plotter.plot_n_vehicles(False)
            plotter.plot_space_time_diagram(['E1_0', 'E1_1', 'E2_0', 'E2_1', 'E2_2', 'E2_3', 'E3_0', 'E3_1', 'E3_2', 'E3_3', 'E4', 'E5_0', 'E5_1', 'E5_2', 'E5_3'], time_range=[0.06944444, 10000], fig_title=label)

            if filename != "nc":
                plotter.plot_rm_rate_queuing(["RM_E7", "RM_E12", "RM_E15"], True)
                plotter.plot_rm_rate_detector_data(["RM_E7", "RM_E12", "RM_E15"], [["E7_down_occ_0", "E7_down_occ_1", "E7_down_occ_2"], ["E12_down_occ_0", "E12_down_occ_1", "E12_down_occ_2"], ["E15_down_occ_0", "E15_down_occ_1", "E15_down_occ_2"]], ["speeds", "occupancies"], data_titles=["Vehicle Speed", "Downstream Occupancy"], aggregate_data=20)
            
            if filename == "lppo" and os.path.exists("ramp_metering/lppo_densities.csv"):
                plot_lppo_density("ramp_metering/lppo_densities.csv")
            elif filename == "cppo" and os.path.exists("ramp_metering/cppo_densities.csv"): plot_cppo_density("ramp_metering/cppo_densities.csv")
            print('finished plots\n')

        #label = "Coordinated PPO Scenario"
        #plotter = Plotter("../dev/data/m_ex/ALINEA/all_alinea.json", sim_label=label, sim_time_units="hr")
        #plotter = Plotter("ramp_metering/new_net_test_cppo.json", sim_label=label, sim_time_units="hr")
        #plotter.plot_rm_rate("RM_E15")
        #plotter.plot_rm_rate_queuing(["RM_E7", "RM_E12", "RM_E15"], True)
        #plotter.plot_space_time_diagram(['E1_0', 'E1_1', 'E2_0', 'E2_1', 'E2_2', 'E3_0', 'E3_1', 'E3_2', 'E4', 'E5_0', 'E5_1', 'E5_2', 'E5_3'], time_range=[200, 300], fig_title=label)#0.06944444, 10000], fig_title=label)
 
        #plotter.plot_rm_rate_detector_data(["RM_E15"], [["E15_down_occ_0", "E15_down_occ_1", "E15_down_occ_2"]], ["speeds", "occupancies"], data_titles=["Vehicle Speed", "Downstream Occupancy"], aggregate_data=30)
        
        #plotter.plot_rm_rate_detector_data(["RM_E7", "RM_E12", "RM_E15"], [["E7_down_occ_0", "E7_down_occ_1", "E7_down_occ_2"], ["E12_down_occ_0", "E12_down_occ_1", "E12_down_occ_2"], ["E15_down_occ_0", "E15_down_occ_1", "E15_down_occ_2"]], ["speeds", "occupancies"], data_titles=["Vehicle Speed", "Downstream Occupancy"], aggregate_data=30)


    if True: # Plot Comparisons

        fps = ["ramp_metering/test2/data/new_net_test_nc.json", "ramp_metering/test2/data/new_net_test_alinea.json", "ramp_metering/test2/data/new_net_test_lppo.json", "ramp_metering/test2/data/new_net_test_cppo.json"]
        labels = ["No Control Scenario", "ALINEA Scenario", "Local PPO Agents Scenario", "Coordinated PPO Agents Scenario"]
        datasets = []
        for fp, label in zip(fps, labels):
            plotter = Plotter(fp)
            #plotter.plot_metering_rates(['RM_E7', 'RM_E15', 'RM_E12'], True, fig_title=label, time_range=[500, 10000])#, 'RM_E15', 'RM_E12'])

            with open(fp, 'r') as file:
                
                dat = json.load(file)
                datasets.append(dat)
                print("loaded", fp)

        #compare_vehicle_data(datasets, "no_vehicles", ["No Control", "ALINEA", "Local PPO", "Coordinated PPO"])
        compare_vehicle_data(datasets, "tts", ["No Control", "ALINEA", "Local PPO", "Coordinated PPO"], cumulative=False)
        compare_vehicle_data(datasets, "delay", ["No Control", "ALINEA", "Local PPO", "Coordinated PPO"], cumulative=False)


    if False: # Plot Simulation Summaries

        label = "ex_v2 ALINEA"
        plotter = Plotter('alinea_sdcsd.json', sim_label=label)
        print('loaded')
        #plotter.plot_rm_rate('RM_E7', time_range=[500, 10000])
        plotter.plot_rm_rate('RM_E15', time_range=[500, 10000])
        plotter.plot_rm_rate('RM_E12', time_range=[500, 10000])
        #plotter.plot_space_time_diagram(['E1_0', 'E1_1', 'E2_0', 'E2_1', 'E2_2', 'E3_0', 'E3_1', 'E3_2', 'E4', 'E5_0', 'E5_1', 'E5_2', 'E5_3'], time_range=[500, 10000], fig_title=label)
 
        exit()

        label = "ex_v2 No Control"
        plotter = Plotter('../dev/data/ex/alinea.json', sim_label=label, fig_save_loc="../dev/figs/ex_v2/")
        print('loaded')
        plotter.plot_metering_rates(['RM_E7', 'RM_E15', 'RM_E12'], True, save_fig="plt")
        exit()
        plotter.plot_n_vehicles(save_fig="n_vehicles")
        plotter.plot_n_vehicles(False, save_fig="cumulative_n_vehicles")
        plotter.plot_space_time_diagram(['E1_0', 'E1_1', 'E2_0', 'E2_1', 'E2_2', 'E3_0', 'E3_1', 'E3_2', 'E4', 'E5_0', 'E5_1', 'E5_2', 'E5_3'], time_range=[0, 10000], fig_title=label, save_fig="st")
 
    if False: # Plot controllers
        plotter5 = Plotter("controller_demo.json")
        #plotter.plot_cumulative_curve(["cw_ramp_inflow", "cw_rm_downstream"], ["cw_rm_upstream"], 15)
        plotter5.plot_meter_queue_length("crooswijk_meter")

        plotter5.plot_space_time_diagram(['321901470', '126729982', '126730069', '126730059', '509506847'])
        plotter5.plot_vsl_data('vsl_0')
        plotter5.plot_rg_data('rg_0')
        plotter5.plot_rm_rate("crooswijk_meter")
        