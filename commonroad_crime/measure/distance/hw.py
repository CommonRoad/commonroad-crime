__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.4.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import matplotlib.pyplot as plt
import logging
import math

import numpy as np

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeDistance, TypeMonotone
from commonroad_crime.measure.time.thw import THW
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.utility.visualization import TUMcolor

from commonroad.geometry.shape import Polygon, Circle

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

    def cal_headway(self, verbose=True):
        if isinstance(self.other_vehicle.obstacle_shape, Polygon):
            other_position = self.other_vehicle.state_at_time(self.time_step).position
        else:
            if isinstance(self.other_vehicle.obstacle_shape, Circle):
                other_position = (
                    self.other_vehicle.state_at_time(self.time_step).position
                    - self.other_vehicle.obstacle_shape.radius
                )
            else:
                other_position = (
                    self.other_vehicle.state_at_time(self.time_step).position
                    - self.other_vehicle.obstacle_shape.length / 2
                )
        ego_position = (
            self.ego_vehicle.state_at_time(self.time_step).position
            + self.ego_vehicle.obstacle_shape.length / 2
        )
        try:
            headway = utils_sol.compute_clcs_distance(
                self.clcs, ego_position, other_position
            )[0]
        except ValueError as e:
            utils_log.print_and_log_warning(
                logger, f"<HW> During the projection of the other vehicle: {e}", verbose
            )
            headway = math.inf
        if headway < 0:
            return math.inf
        else:
            return headway

    def visualize(self, figsize: tuple = (25, 15)):
        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(
                self.time_step,
                self.ego_vehicle.prediction.trajectory.state_list,
                margin=10,
            )

        self._initialize_vis(figsize=figsize, plot_limit=plot_limits)
        self.rnd.render()
        utils_vis.draw_reference_path(self.rnd, np.array(self.clcs.reference_path()))
        utils_vis.draw_state_list(
            self.rnd,
            self.ego_vehicle.prediction.trajectory.state_list[self.time_step :],
            color=TUMcolor.TUMblue,
            linewidth=5,
        )
        utils_vis.draw_state(
            self.rnd,
            self.ego_vehicle.state_at_time(self.time_step),
            color=TUMcolor.TUMblue,
        )
        utils_vis.draw_dyn_vehicle_shape(
            self.rnd, self.other_vehicle, self.time_step, color=TUMcolor.TUMred
        )
        plt.title(f"{self.measure_name} of {self.value} m")
        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(
                    self.measure_name,
                    self.configuration.general.path_output,
                    self.time_step,
                )
            else:
                plt.show()
