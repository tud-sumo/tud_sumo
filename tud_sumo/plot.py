import json, math, csv, matplotlib.pyplot as plt, numpy as np
import matplotlib.patheffects as pe
import matplotlib
from mpl_toolkits.axes_grid1 import make_axes_locatable
import os.path
from copy import deepcopy
from simulation import Simulation
from utils import *

default_labels = {"no_vehicles": "No. of Vehicles", "tts": "Total Time Spent (s)", "delay": "Delay (s)", "throughput": "Throughput (veh/hr)",
                  "veh_counts": "No. of Vehicles", "occupancies": "Occupancy (%)", "densities": "Density unit",
                  "nc": "No Control", "alinea": "ALINEA", "lppo": "Local PPO", "cppo": "Coordinated PPO", "ppo": "Proximal Policy Optimisation (PPO)"}

default_titles = {"no_vehicles": "Number of Vehicles", "tts": "Total Time Spent", "delay": "Delay",
                  "veh_counts": "Number of Vehicles", "occupancies": "Vehicle Occupancies", "densities": "Vehicle Density",
                  "speeds": "Average Speed", "limits": "Speed Limit", "throughput": "Throughput"}

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
    
    def display_figure(self, filename: str|None=None, save_dpi: int=600) -> None:
        """
        Display figure, either saving to file or showing on screen.
        :param filename: Save file name, if saving
        :param save_dpi: Plot image dpi, if saving
        """

        if filename is None: plt.show()
        else:
            
            if not filename.endswith(".png") and not filename.endswith('.jpg'):
                filename += ".png"

            fp = self.fig_save_loc + filename
            if os.path.exists(fp) and not self.overwrite_figs:
                raise FileExistsError("Plotter.display_figure() File '{0}' already exists.".format(fp))
            
            plt.savefig(fp, dpi=600)

        plt.close()
        
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
                    if "detectors" in self.sim_data["data"].keys():
                        if det_id in self.sim_data["data"]["detectors"].keys():
                            if data_key in self.sim_data["data"]["detectors"][det_id].keys():
                                det_data = self.sim_data["data"]["detectors"][det_id][data_key]
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

    def plot_rm_queuing(self, rm_id: str, ax=None, yax_labels: bool|list|tuple=True, xax_labels: bool=True, plot_delay: bool=True, cumulative_delay: bool=False, time_range: list|tuple|None=None, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot ramp metering rate.
        :param rm_id:            Ramp meter junction ID
        :param ax:               Matplotlib axis, used when creating subplots
        :param yax_labels:       Bool denoting whether to include y-axis labels (for subplots). Either single bool for both y-axis labels or list of two bools to set both y-axes (when plotting delay).
        :param xax_labels:       Bool denoting whether to include x-axis labels (for subplots)
        :param plot_delay:       Bool denoting whether to plot queue delay. This will be done on the same plot with a separate y-axis.
        :param cumulative_delay: Bool denoting whether to plot cumulative delay
        :param time_range:       Plotting time range (in plotter class units)
        :param show_events:      Bool denoting whether to plot events on all axes
        :param fig_title:        If given, will overwrite default title
        :param save_fig:         Output image filename, will show image if not given
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
                if plot_delay and cumulative_delay: default_title += " & Cumulative Delay"
                elif plot_delay: default_title += " & Delay"
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
            if cumulative_delay: queue_delays = get_cumulative_arr(queue_delays)
            ax2 = ax1.twinx()

            ax2.plot(data_time_vals, queue_delays, linewidth=1.5, zorder=3, color=colour)
            ax2.tick_params(axis='y', labelcolor=colour)
            if (isinstance(yax_labels, bool) and yax_labels) or (isinstance(yax_labels, (list, tuple)) and len(yax_labels) == 2 and yax_labels[1]):
                if cumulative_delay: ax2.set_ylabel("Cumulative Delay (s)", color=colour)
                else: ax2.set_ylabel("Delay (s)", color=colour)
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
                self.plot_rm_queuing(rm_ids[0], ax2, True, True, True, False, time_range, show_events, fig_title="Queue Lengths & Delays")
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
                    self.plot_rm_queuing(rm_id, axes[1][idx], (idx==0, idx==len(rm_ids)-1), True, True, False, time_range, show_events, "")

        def_title = "Ramp Metering Rates"
        if plot_queuing: def_title += " & Queuing Data"
        fig_title = self.sim_label+def_title if not isinstance(fig_title, str) else fig_title
        if fig_title != "": fig.suptitle(fig_title, fontweight='bold')

        fig.tight_layout()
        self.display_figure(save_fig)

    def plot_vehicle_data(self, data_key: str, plot_cumulative: bool=False, time_range: list|tuple|None=None, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot network-wide vehicle data.
        :param data_key:        Data key to plot, either "no_vehicles", "tts" or "delay"
        :param plot_cumulative: Bool denoting whether to plot cumulative values
        :param time_range:      Plotting time range (in plotter class units)
        :param show_events:     Bool denoting whether to plot when events occur
        :param fig_title:       If given, will overwrite default title
        :param save_fig:        Output image filename, will show image if not given
        """

        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        if data_key not in ["no_vehicles", "tts", "delay"]:
            raise KeyError("Plotter.plot_vehicle_data(): Unrecognised data key '{0}' (must be ['no_vehicles'|'tts'|'delay']).".format(data_key))

        fig, ax = plt.subplots(1, 1)
        start, step = self.sim_data["start"], self.sim_data["step_len"]

        y_vals = self.sim_data["data"]["vehicles"][data_key]
        if plot_cumulative: y_vals = get_cumulative_arr(y_vals)
        x_vals = get_time_steps(y_vals, self.sim_time_units, step, start)
        x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)

        ax.plot(x_vals, y_vals)

        if fig_title == None:
            fig_title = default_titles[data_key]
            if plot_cumulative: fig_title = "Cumulative "+fig_title
            fig_title = self.sim_label + fig_title
        ax.set_title(fig_title, pad=20)

        ax.set_xlabel(default_labels["sim_time"])
        ax.set_ylabel(default_labels[data_key])
        ax.set_xlim([x_vals[0], x_vals[-1]])
        ax.set_ylim([0, get_axis_lim(y_vals)])
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)
        
        fig.tight_layout()

        self.display_figure(save_fig)

    def plot_detector_data(self, detector_id: str, data_key: str, plot_cumulative: bool=False, time_range: list|tuple|None=None, show_events: bool=True, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plot detector data.
        :param detector_id:     Detector ID
        :param data_key:        Data key to plot, either "speeds", "veh_counts" or "occupancies"
        :param plot_cumulative: Bool denoting whether to plot cumulative values
        :param time_range:      Plotting time range (in plotter class units)
        :param show_events:     Bool denoting whether to plot when events occur
        :param fig_title:       If given, will overwrite default title
        :param save_fig:        Output image filename, will show image if not given
        """
        
        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        fig, ax = plt.subplots(1, 1)
        start, step = self.sim_data["start"], self.sim_data["step_len"]

        if data_key not in ["speeds", "veh_counts", "occupancies"]:
            raise KeyError("Plotter.plot_detector_data(): Unrecognised data key '{0}' (must be [speeds|veh_counts|occupancies]).".format(data_key))
        elif detector_id not in self.sim_data["data"]["detectors"].keys():
            raise KeyError("Plotter.plot_detector_data(): Detector ID '{0}' not found.".format(detector_id))
        elif data_key == "occupancy" and self.sim_data["data"]["detectors"][detector_id]["type"] == "multientryexit":
            raise ValueError("Plotter.plot_detector_data(): Multi-Entry-Exit Detectors ('{0}') do not measure '{1}'.".format(detector_id, data_key))
        
        y_vals = self.sim_data["data"]["detectors"][detector_id][data_key]
        if plot_cumulative: y_vals = get_cumulative_arr(y_vals)
        x_vals = get_time_steps(y_vals, self.sim_time_units, step, start)
        x_vals, y_vals = limit_vals_by_range(x_vals, y_vals, time_range)

        ax.plot(x_vals, y_vals)

        if fig_title == None:
            fig_title = "{0} (Detector '{1}')".format(default_titles[data_key], detector_id)
            if plot_cumulative: fig_title = "Cumulative "+fig_title
            fig_title = self.sim_label + fig_title
        ax.set_title(fig_title, pad=20)

        ax.set_xlabel(default_labels["sim_time"])
        ax.set_ylabel(default_labels[data_key])
        ax.set_xlim([x_vals[0], x_vals[-1]])
        if data_key == "occupancies": ax.set_ylim([0, 100])
        else: ax.set_ylim([0, get_axis_lim(y_vals)])
        ax.grid(True, 'both', color='grey', linestyle='-', linewidth=0.5)

        if "events" in self.sim_data["data"].keys() and show_events:
            if "completed" in self.sim_data["data"]["events"]:
                self.plot_event(ax)
        
        fig.tight_layout()

        self.display_figure(save_fig)

    def plot_od_trip_times(self, od_pairs: list|tuple|None=None, vtypes: list|tuple|None=None, ascending_vals: bool=True, time_unit: str="m", time_range: list|tuple|None=None, fig_title: str|None=None, save_fig: str|None=None) -> None:
        """
        Plots average trip times for Origin-Destination pairs.
        :param od_pairs:       (n x 2) list containing OD pairs. If not given, all OD pairs are plotted
        :param vtypes:         List of vehicle types for included trips (defaults to all)
        :param ascending_vals: If true, the largest values are plotted in the bottom-right, if false, top-left
        :param time_unit:      Time unit for displaying values, must be ['s'|'m'|'hr'], defaults to 'm'
        :param time_range:     Plotting time range (in plotter class units, separate to time_unit parameter)
        :param fig_title:      If given, will overwrite default title
        :param save_fig:       Output image filename, will show image if not given
        """
        
        if time_unit not in ["s", "m", "hr"]:
            raise ValueError("Invalid time unit '{0}' (must be ['s'|'m'|'hr']).".format(time_unit))
        
        if self.simulation != None:
            self.sim_data = self.simulation.all_data
            self.units = self.simulation.units.name

        step = self.sim_data["step_len"]

        od_trip_times, add_new = {}, od_pairs == None
        all_origins, all_destinations = set([]), set([])
        if od_pairs != None:
            for pair in od_pairs:
                od_trip_times[pair[0]] = {pair[1]: []}
                all_origins.add(pair[0])
                all_destinations.add(pair[1])

        n_trips = 0
        com_trip_data = self.sim_data["data"]["trips"]["completed"]
        for trip in com_trip_data.values():
            origin, destination = trip["origin"], trip["destination"]
            veh_type = trip["vehicle_type"]

            if vtypes != None and veh_type not in veh_type: continue
            
            if origin not in od_trip_times.keys():
                if add_new:
                    od_trip_times[origin] = {}
                else: continue
            
            if destination not in od_trip_times[origin].keys():
                if add_new: od_trip_times[origin][destination] = []
                else: continue

            trip_time = convert_time_units(trip["arrival"] - trip["departure"], time_unit, step)
            trip_departure, trip_arrival = convert_time_units([trip["departure"], trip["arrival"]], self.sim_time_units, step)

            if time_range != None and trip_departure < time_range[0] and trip_arrival > time_range[1]:
                continue

            od_trip_times[origin][destination].append(trip_time)
            all_origins.add(origin)
            all_destinations.add(destination)
            n_trips += 1
        
        if n_trips == 0:
            raise ValueError("No trips found.")

        all_origins = list(all_origins)
        all_destinations = list(all_destinations)

        if add_new:
            avg_o_tts = []
            for o in all_origins:
                o_tts = [sum(d)/len(d) for d in od_trip_times[o].values()]
                avg_o_tts.append(sum(o_tts)/len(o_tts))
            
            all_origins = [x for _, x in sorted(zip(avg_o_tts, all_origins))]
            
            avg_d_tts = []
            for d in all_destinations:
                d_tts = []
                for o_data in od_trip_times.values():
                    if d in o_data.keys():
                        d_tts.append(sum(o_data[d])/len(o_data[d]))
                avg_d_tts.append(sum(d_tts)/len(d_tts))

            all_destinations = [x for _, x in sorted(zip(avg_d_tts, all_destinations))]

        att_matrix = np.empty((len(all_origins), len(all_destinations)))
        att_matrix[:] = np.nan
        
        if add_new and not ascending_vals:
            all_origins.reverse()
            all_destinations.reverse()

        for i, origin in enumerate(all_origins):
            for j, destination in enumerate(all_destinations):
                if destination in od_trip_times[origin].keys():
                    trip_times = od_trip_times[origin][destination]
                    att_matrix[i][j] = sum(trip_times) / len(trip_times)

        fig, ax = plt.subplots(1, 1, figsize=(6, 6))
        masked_array = np.ma.array(att_matrix, mask=np.isnan(att_matrix))
        cmap = matplotlib.cm.Reds
        cmap.set_bad('#f7f7f7')
        ax.matshow(masked_array, interpolation='nearest', cmap=cmap)

        ax.set_xticks(np.arange(len(all_destinations)), labels=all_destinations)
        ax.set_yticks(np.arange(len(all_origins)), labels=all_origins)
        ax.xaxis.set_ticks_position("bottom")
        ax.xaxis.set_label_position("top")
        ax.yaxis.set_label_position("right")

        for row in range(att_matrix.shape[0]):
            for col in range(att_matrix.shape[1]):
                if not np.isnan(att_matrix[row, col]):
                    ax.text(x=col, y=row, s=round(att_matrix[row, col], 2) if time_unit != "s" else int(att_matrix[row, col]),
                            va='center', ha='center', color='white', path_effects=[pe.withStroke(linewidth=2, foreground="black")]) 

        plt.setp(ax.get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")

        ax.set_xlabel("Destination ID")
        ax.set_ylabel("Origin ID")
        
        if fig_title == None:
            fig_title = self.sim_label + "Average Trip Times in {0}".format(time_desc[time_unit])
        ax.set_title(fig_title, pad=30, fontweight='bold')

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
        start, end, step = self.sim_data["start"], self.sim_data["end"], self.sim_data["step_len"]

        if inflow_detectors == None and outflow_detectors == None:

            inflows, outflows = [0] * (end - start), [0] * (end - start)

            trips = self.sim_data["data"]["trips"]
            for inc_trip in trips["incomplete"].values():
                inflows[inc_trip["departure"]] += 1
            
            for com_trip in trips["completed"].values():
                inflows[com_trip["departure"]] += 1
                outflows[com_trip["arrival"]] += 1

        else:
            if inflow_detectors == None or outflow_detectors == None:
                raise TypeError("Plotter.plot_cumulative_curve(): If using detectors, both inflow and outflow detectors are required.")
            
            if "detectors" not in self.sim_data["data"].keys():
                raise KeyError("Plotter.plot_vehicle_detector_data(): No detector data to plot.")
            
            detector_data = self.sim_data["data"]["detectors"]
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
        x_vals = get_time_steps(inflows, self.sim_time_units, step, start)

        fig, ax = plt.subplots(1, 1)
        fig_title = "{0}Cumulative Arrival-Departure Curve".format(self.sim_label) if not isinstance(fig_title, str) else fig_title
        ax.set_title(fig_title, pad=20)
        
        ax.plot(x_vals, inflows, label="Inflow", zorder=3)
        ax.plot(x_vals, outflows, label="Outflow", zorder=4)
        ax.set_xlim([x_vals[0], x_vals[-1]])
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
        total_len = sum([self.sim_data["data"]["edges"][e_id]["length"] for e_id in edge_ids])

        if self.units in ["METRIC", "UK"]:
            if total_len >= 1000: y_scale, y_label = 0.001, "Distance (km)"
            else: y_label = "Distance (m)"
        elif self.units in ["IMPERIAL"]:
            y_label = "Distance (mi)"
        x_label = default_labels["sim_time"]

        if time_range == None: time_range = [-math.inf, math.inf]

        ordered_points = {}
        for e_id in edge_ids:
            if e_id not in self.sim_data["data"]["edges"].keys():
                raise KeyError("Plotter.plot_space_time_diagram(): Edge '{0}' not found in tracked edges.".format(e_id))
            else: e_data = self.sim_data["data"]["edges"][e_id]

            step_vehicles, edge_length = e_data["step_vehicles"], e_data["length"]
            start, step = e_data["init_time"], self.sim_data["step_len"]

            curr_step = start

            for step_data in step_vehicles:
                curr_time = convert_time_units(curr_step, self.sim_time_units, step)
                if curr_time <= time_range[1] and curr_time >= time_range[0]:
                    for veh_data in step_data:

                        y_val = (veh_data[1] * edge_length) + edge_offset
                        if not upstream_at_top: y_val = total_len - y_val
                        y_val *= y_scale

                        if curr_step not in ordered_points.keys():
                            ordered_points[curr_step] = [(y_val, veh_data[2])]
                        else: ordered_points[curr_step].append((y_val, veh_data[2]))
                        
                elif curr_time > time_range[1]:
                    break

                curr_step += 1

            edge_offset += edge_length

        idxs = ordered_points.keys()
        x_vals, y_vals, speed_vals = [], [], []
        for idx in idxs:
            x_vals += [convert_time_units(idx, self.sim_time_units, step)] * len(ordered_points[idx])
            dist_speed = ordered_points[idx]
            y_vals += [val[0] for val in dist_speed]
            speed_vals += [val[1] for val in dist_speed]

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
