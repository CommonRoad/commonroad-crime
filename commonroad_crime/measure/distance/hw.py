__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import matplotlib.pyplot as plt
import logging
import math

import numpy as np

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeDistance, TypeMonotone
from commonroad_crime.measure.time.thw import THW
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.solver as utils_sol
from commonroad_crime.utility.visualization import TUMcolor

from commonroad.geometry.shape import Polygon

logger = logging.getLogger(__name__)


class HW(THW):
    """
    https://criticality-metrics.readthedocs.io/en/latest/time-scale/THW.html
    This is taken as a inherited class from time headway
    """
    measure_name = TypeDistance.HW
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(HW, self).__init__(config)

    def cal_headway(self):
        if isinstance(self.other_vehicle.obstacle_shape, Polygon):
            # todo: fix for ttz
            other_position = self.other_vehicle.state_at_time(self.time_step).position
        else:
            other_position = self.other_vehicle.state_at_time(self.time_step).position - \
                             self.other_vehicle.obstacle_shape.length / 2
        ego_position = self.ego_vehicle.state_at_time(self.time_step).position + \
                       self.ego_vehicle.obstacle_shape.length / 2
        headway = utils_sol.compute_clcs_distance(self.clcs, ego_position, other_position)[0]
        if headway < 0:
            return math.inf
        else:
            return headway

    def visualize(self, figsize: tuple = (25, 15)):
        self._initialize_vis(figsize=figsize,
                             plot_limit=utils_vis.
                             plot_limits_from_state_list(self.time_step,
                                                         self.ego_vehicle.prediction.trajectory.state_list,
                                                         margin=10))
        self.rnd.render()
        utils_vis.draw_reference_path(self.rnd, np.array(self.clcs.reference_path()))
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMblue, linewidth=5)
        utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(self.time_step), color=TUMcolor.TUMblue)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, self.time_step, color=TUMcolor.TUMred)
        plt.title(f"{self.measure_name} of {self.value} m")
        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()
