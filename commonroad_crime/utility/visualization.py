from pathlib import Path
from typing import Union, List
from enum import Enum
import numpy as np
import matplotlib.pyplot as plt

from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.scenario import State, Scenario
from commonroad.scenario.obstacle import DynamicObstacle

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.scene import Scene

zorder = 22


class TUMcolor(tuple, Enum):
    TUMblue = (0, 101 / 255, 189 / 255)
    TUMred = (227 / 255, 27 / 255, 35 / 255)
    TUMgreen = (162 / 255, 173 / 255, 0)
    TUMgray = (156 / 255, 157 / 255, 159 / 255)
    TUMdarkgray = (88 / 255, 88 / 255, 99 / 255)
    TUMorange = (227 / 255, 114 / 255, 34 / 255)
    TUMdarkblue = (0, 82 / 255, 147 / 255)
    TUMwhite = (1, 1, 1)
    TUMblack = (0, 0, 0)
    TUMlightgray = (217 / 255, 218 / 255, 219 / 255)


def save_fig(metric_name: str, path_output: str, time_step: Union[int, float]):
    # save as svg
    Path(path_output).mkdir(parents=True, exist_ok=True)
    plt.savefig(f'{path_output}{metric_name}_{time_step:.0f}.svg', format="svg", bbox_inches="tight",
                transparent=False)


def plot_limits_from_state_list(time_step: int, state_list: List[State], margin: float = 10.0):
    return [state_list[time_step].position[0] - margin,
            state_list[-1].position[0] + margin,
            state_list[time_step].position[1] - margin,
            state_list[time_step].position[1] + margin]


def draw_state(rnd: MPRenderer, state: State, color: TUMcolor = TUMcolor.TUMgreen):
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
    x, y = obstacle.occupancy_at_time(time_step).shape.shapely_object.exterior.xy
    rnd.ax.fill(x, y, alpha=0.5, fc=color, ec=None, zorder=zorder)
    zorder += 1


def draw_circle(rnd: MPRenderer, center: np.ndarray, radius: float,
                opacity: float = 0.5, color: TUMcolor = TUMcolor.TUMblue):
    global zorder
    cir = plt.Circle((center[0], center[1]), radius, color=color, zorder=zorder, alpha=opacity)
    zorder += 1
    rnd.ax.add_patch(cir)


def draw_state_list(rnd: MPRenderer, state_list: List[State],
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
    rnd.ax.plot(pos[:, 0], pos[:, 1], color=color, markersize=1.5,
                zorder=zorder, linewidth=linewidth, alpha=opacity)
    zorder += 1


def draw_sce_at_time_step(rnd: MPRenderer,
                          config: CriMeConfiguration,
                          sce: Union[Scenario, Scene],
                          time_step: int):
    sce.draw(rnd, draw_params={'time_begin': time_step,
                               "trajectory": {
                                   "draw_trajectory": False},
                               "dynamic_obstacle": {
                                   "draw_icon": config.debug.draw_icons,
                               },
                               "static_obstacle": {
                                   "occupancy": {
                                       "shape": {
                                           "polygon": {
                                               "facecolor": TUMcolor.TUMgray,
                                               "edgecolor": TUMcolor.TUMdarkgray,
                                           },
                                           "rectangle": {
                                               "facecolor": TUMcolor.TUMgray,
                                               "edgecolor": TUMcolor.TUMdarkgray,
                                           },
                                           "circle": {
                                               "facecolor": TUMcolor.TUMgray,
                                               "edgecolor": TUMcolor.TUMdarkgray,
                                           }
                                       }

                                   }
                               },
                               "lanelet": {
                                   "fill_lanelet": False,
                               }
                               })