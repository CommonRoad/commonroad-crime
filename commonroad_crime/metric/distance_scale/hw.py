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
from commonroad_crime.data_structure.type import TypeDistanceScale
from commonroad_crime.metric.time_scale.thw import THW
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class HW(THW):
    """
    https://criticality-metrics.readthedocs.io/en/latest/time-scale/THW.html
    This is taken as a inherited class from time headway
    """
    metric_name = TypeDistanceScale.HW

    def __init__(self, config: CriMeConfiguration):
        super(HW, self).__init__(config)

    def cal_headway(self):
        other_position = self.other_vehicle.state_at_time(self.time_step).position
        other_s, _ = self.clcs.convert_to_curvilinear_coords(other_position[0], other_position[1])
        ego_position = self.ego_vehicle.state_at_time(self.time_step).position
        ego_s, _ = self.clcs.convert_to_curvilinear_coords(ego_position[0], ego_position[1])
        return other_s - ego_s

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
        plt.title(f"{self.metric_name} of {self.value} m")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()
