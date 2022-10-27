from pathlib import Path
from typing import Union
from enum import Enum
import matplotlib.pyplot as plt

from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.scenario import State, Scenario
from commonroad.scenario.obstacle import DynamicObstacle

from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.scene import Scene


class TUMcolor(tuple, Enum):
    TUMblue = (0, 101 / 255, 189 / 255)
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


def draw_state(rnd: MPRenderer, state: State, color: str = TUMcolor.TUMgreen, flag_save: bool = True):
    if flag_save:
        s_c = 1.2
        s_b = 2.2
    else:
        s_c = 120
        s_b = 180
    # the cut-off state
    rnd.ax.scatter(state.position[0], state.position[1], marker='o', color=color, edgecolor='none', zorder=30, s=s_c)
    rnd.ax.scatter(state.position[0], state.position[1], marker='o', color=TUMcolor.TUMwhite, edgecolor='none',
                   zorder=29, s=s_b)


def draw_dyn_vehicle_shape(rnd: MPRenderer, obstacle: DynamicObstacle, time_step: int, color: str = TUMcolor.TUMblue):
    x, y = obstacle.occupancy_at_time(time_step).shape.shapely_object.exterior.xy
    rnd.ax.fill(x, y, alpha=0.5, fc=color, ec=None, zorder=24)


def draw_sce_at_time_step(rnd: MPRenderer,
                          config: CriticalityConfiguration,
                          sce: Union[Scenario, Scene],
                          time_step: int):
    sce.draw(rnd, draw_params={'time_begin': time_step,
                               "dynamic_obstacle": {
                                   "draw_icon": config.debug.draw_icons},
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

