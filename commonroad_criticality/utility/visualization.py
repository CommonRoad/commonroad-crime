from pathlib import Path
from typing import Union
import matplotlib.pyplot as plt

from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.scenario import State, Scenario

from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.scene import Scene


def save_fig(metric_name: str, path_output: str, time_step: int):
    # save as svg
    Path(path_output).mkdir(parents=True, exist_ok=True)
    plt.savefig(f'{path_output}{metric_name}_{time_step:.0f}.svg', format="svg", bbox_inches="tight",
                transparent=False)


def draw_cut_off_state(rnd: MPRenderer, state: State):
    # the cut-off state
    rnd.ax.scatter(state.position[0], state.position[1], marker='o', color='#ffc325ff', edgecolor='none', zorder=30,
                   s=1.2)
    rnd.ax.scatter(state.position[0], state.position[1], marker='o', color='w', edgecolor='none', zorder=29, s=2.2)


def draw_sce_at_time_step(rnd: MPRenderer,
                          config: CriticalityConfiguration,
                          sce: Union[Scenario, Scene],
                          time_step: int):
    sce.draw(rnd, draw_params={'time_begin': time_step,
                               "dynamic_obstacle": {
                                   "draw_icon": config.debug.draw_icons}})
    rnd.render()
