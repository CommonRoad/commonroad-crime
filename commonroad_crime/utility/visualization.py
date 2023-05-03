__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
from pathlib import Path
from typing import Union, List
from enum import Enum
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import ShapeGroup
from commonroad.scenario.state import PMState
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import DynamicObstacle

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.scene import Scene
from commonroad_crime.data_structure.type import TypeMonotone


class TUMcolor(tuple, Enum):
    TUMblue = (0, 101 / 255, 189 / 255)
    TUMred = (227 / 255, 27 / 255, 35 / 255)
    TUMdarkred = (139 / 255, 0, 0)
    TUMgreen = (162 / 255, 173 / 255, 0)
    TUMgray = (156 / 255, 157 / 255, 159 / 255)
    TUMdarkgray = (88 / 255, 88 / 255, 99 / 255)
    TUMorange = (227 / 255, 114 / 255, 34 / 255)
    TUMdarkblue = (0, 82 / 255, 147 / 255)
    TUMwhite = (1, 1, 1)
    TUMblack = (0, 0, 0)
    TUMlightgray = (217 / 255, 218 / 255, 219 / 255)


zorder = 22



def save_fig(metric_name: str, path_output: str, time_step: Union[int, float]):
    # save as svg
    Path(path_output).mkdir(parents=True, exist_ok=True)
    plt.savefig(f'{path_output}{metric_name}_{time_step:.0f}.svg', format="svg", bbox_inches="tight",
                transparent=False)


def plot_limits_from_state_list(time_step: int, state_list: List[PMState], margin: float = 10.0):
    return [state_list[time_step].position[0] - margin,
            state_list[-1].position[0] + margin,
            state_list[time_step].position[1] - margin / 2,
            state_list[time_step].position[1] + margin / 2]


def draw_state(rnd: MPRenderer, state: PMState, color: TUMcolor = TUMcolor.TUMgreen):
    global zorder
    cir_c = plt.Circle((state.position[0], state.position[1]), 0.12, color=color,
                       linewidth=10., zorder=zorder)
    cir_b = plt.Circle((state.position[0], state.position[1]), 0.2, color=TUMcolor.TUMwhite,
                       linewidth=10., zorder=zorder - 1)
    zorder += 1
    rnd.ax.add_patch(cir_c)
    rnd.ax.add_patch(cir_b)


def draw_dyn_vehicle_shape(rnd: MPRenderer, obstacle: DynamicObstacle, time_step: int,
                           color: TUMcolor = TUMcolor.TUMblue):
    global zorder
    obs_shape = obstacle.occupancy_at_time(time_step).shape
    if isinstance(obs_shape, ShapeGroup):
        for shape_element in obs_shape.shapes:
            x, y = shape_element.shapely_object.exterior.xy
            rnd.ax.fill(x, y, alpha=0.5, fc=color, ec=None, zorder=zorder)
    else:
        x, y = obs_shape.shapely_object.exterior.xy
        rnd.ax.fill(x, y, alpha=0.5, fc=color, ec=None, zorder=zorder)
    zorder += 1


def draw_circle(rnd: MPRenderer, center: np.ndarray, radius: float,
                opacity: float = 0.5, color: TUMcolor = TUMcolor.TUMblue):
    global zorder
    cir = plt.Circle((center[0], center[1]), radius, color=color, zorder=zorder, alpha=opacity)
    zorder += 1
    rnd.ax.add_patch(cir)


def draw_reference_path(rnd: MPRenderer, ref_path: np.ndarray, color: TUMcolor = TUMcolor.TUMorange):
    global zorder
    rnd.ax.plot(ref_path[:, 0], ref_path[:, 1], color=color,
                marker='.', markersize=1, zorder=zorder, linewidth=1.,
                label='reference path')
    zorder += 1


def draw_state_list(rnd: MPRenderer, state_list: List[PMState],
                    start_time_step: Union[None, int] = None,
                    color: TUMcolor = TUMcolor.TUMdarkblue,
                    linewidth: float = 0.75) -> None:
    """
    Visualizing the state list as a connecting trajectory. The transparency is based on the starting
    time step.
    """
    global zorder
    # visualize optimal trajectory
    pos = np.asarray([state.position for state in state_list])
    if start_time_step:
        opacity = 0.5 * (start_time_step / len(state_list) + 1)
    else:
        opacity = 1
    rnd.ax.plot(pos[:, 0], pos[:, 1], linestyle='-', marker='o', color=color, markersize=5,
                zorder=zorder, linewidth=linewidth, alpha=opacity)
    zorder += 1


def draw_sce_at_time_step(rnd: MPRenderer,
                          config: CriMeConfiguration,
                          sce: Union[Scenario, Scene],
                          time_step: int):
    rnd.draw_params.time_begin = time_step
    rnd.draw_params.trajectory.draw_trajectory = False
    rnd.draw_params.dynamic_obstacle.draw_icon = config.debug.draw_icons
    rnd.draw_params.static_obstacle.occupancy.shape.edgecolor = TUMcolor.TUMdarkgray
    rnd.draw_params.static_obstacle.occupancy.shape.facecolor = TUMcolor.TUMgray
    rnd.draw_params.lanelet_network.lanelet.fill_lanelet = False
    sce.draw(rnd)


def plot_criticality_curve(crime, nr_per_row=2, flag_latex=True):
    if flag_latex:
        # use Latex font
        FONTSIZE = 28
        plt.rcParams['text.latex.preamble'] = [r"\usepackage{lmodern}"]
        pgf_with_latex = {  # setup matplotlib to use latex for output
            "pgf.texsystem": "pdflatex",  # change this if using xetex or lautex
            "text.usetex": True,  # use LaTeX to write all text
            "font.family": 'lmodern',
            # blank entries should cause plots
            "font.sans-serif": [],  # ['Avant Garde'],              # to inherit fonts from the document
            # 'text.latex.unicode': True,
            "font.monospace": [],
            "axes.labelsize": FONTSIZE,  # LaTeX default is 10pt font.
            "font.size": FONTSIZE - 10,
            "legend.fontsize": FONTSIZE,  # Make the legend/label fonts
            "xtick.labelsize": FONTSIZE,  # a little smaller
            "ytick.labelsize": FONTSIZE,
            "pgf.preamble": [
                r"\usepackage[utf8x]{inputenc}",  # use utf8 fonts
                r"\usepackage[T1]{fontenc}",  # plots will be generated
                r"\usepackage[detect-all,locale=DE]{siunitx}",
            ]  # using this preamble
        }
        matplotlib.rcParams.update(pgf_with_latex)
    if crime.measures is not None and crime.time_start is not None and crime.time_end is not None:
        nr_metrics = len(crime.measures)
        if nr_metrics > nr_per_row:
            nr_column = nr_per_row
            nr_row = math.ceil(nr_metrics / nr_column)
        else:
            nr_column = nr_metrics
            nr_row = 1
        fig, axs = plt.subplots(nr_row, nr_column, figsize=(7.5 * nr_column, 5 * nr_row))
        count_row, count_column = 0, 0
        for measure in crime.measures:
            criticality_list = []
            time_list = []
            for time_step in range(crime.time_start, crime.time_end + 1):
                if measure.measure_name.value in crime.criticality_dict[time_step]:
                    criticality_list.append(crime.criticality_dict[time_step][measure.measure_name.value])
                    time_list.append(time_step)
            if nr_metrics == 1:
                ax = axs
            elif nr_row == 1:
                ax = axs[count_column]
            else:
                ax = axs[count_row, count_column]
            ax.plot(time_list, criticality_list)
            ax.axis(xmin=time_list[0], xmax=time_list[-1])
            ax.title.set_text(measure.measure_name.value)

            if measure.monotone == TypeMonotone.NEG:
                ax.invert_yaxis()
            count_column += 1
            if count_column > nr_per_row - 1:
                count_column = 0
                count_row += 1
        plt.show()


def visualize_scenario_at_time_steps(scenario: Scenario, plot_limit, time_steps: List[int]):
    rnd = MPRenderer(plot_limits=plot_limit)
    rnd.draw_params.time_begin = time_steps[0]
    if time_steps:
        rnd.draw_params.time_end = time_steps[-1]
    rnd.draw_params.trajectory.draw_trajectory = False
    rnd.draw_params.dynamic_obstacle.draw_icon = True
    scenario.draw(rnd)
    rnd.render()
    for obs in scenario.obstacles:
        draw_state_list(rnd, obs.prediction.trajectory.state_list[time_steps[0]:time_steps[-1] + 1],
                        color=TUMcolor.TUMblue, linewidth=5)
        for ts in time_steps[1:]:
            draw_dyn_vehicle_shape(rnd, obs, ts, color=TUMcolor.TUMblue)
    plt.show()
