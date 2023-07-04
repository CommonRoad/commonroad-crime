__author__ = "Yuanfei Lin, Liguo Chen"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import matplotlib.pyplot as plt
import logging
import math

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
from commonroad_crime.measure.time.et import ET, create_polygon
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor
from commonroad.scenario.scenario import Tag
from commonroad.scenario.obstacle import DynamicObstacle

logger = logging.getLogger(__name__)


class PET(ET):
    """
    See https://criticality-metrics.readthedocs.io/
    """
    measure_name = TypeTime.PET
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(PET, self).__init__(config)
        self.ca = None  # ca stands for conflict area
        self.other_vehicle_exit_time = None
        self.other_vehicle_enter_time = None
        self.ego_vehicle_exit_time = None
        self.ego_vehicle_enter_time = None

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} beginning at time step {time_step}")
        self.time_step = time_step
        self.set_other_vehicles(vehicle_id)
        if (Tag.INTERSECTION not in self.sce.tags) or (len(self.sce.lanelet_network.intersections) == 0):
            utils_log.print_and_log_info(logger, f"* \t\tMeasure only for intersection. PET is set to inf.")
            self.value = math.inf
            return self.value
        if not isinstance(self.other_vehicle, DynamicObstacle):
            utils_log.print_and_log_info(logger,
                                         f"*\t\t {self.other_vehicle} Not a dynamic obstacle, PET is set to inf")
            self.value = math.inf
            return self.value
        # Create an agent object of ET to obtain the conflict area
        self.ca = self.get_ca()
        self.value, self.other_vehicle_enter_time, self.other_vehicle_exit_time = (
            self.get_ca_duration(self.other_vehicle, self.time_step, self.ca)
        )
        _, self.ego_vehicle_enter_time, self.ego_vehicle_exit_time = (
            self.get_ca_duration(self.ego_vehicle, self.time_step, self.ca)
        )
        # The conflict area may not exist, indicated by self.ca being None.
        # Even if the conflict area exists, there are two scenarios where the PET remains undefined,
        # and we set it to infinity. This information is logged in info_value_not_exit()
        self.info_value_not_exist()
        return self.value

    def visualize(self, figsize: tuple = (25, 15)):
        if self.ca is None:
            utils_log.print_and_log_info(logger, "* \t\tNo conflict area")
            return 0
        if self.other_vehicle_exit_time is None and self.other_vehicle_enter_time is None:
            utils_log.print_and_log_info(logger, "* \t\tNo conflict area")
            return 0
        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(self.time_step,
                                                                self.ego_vehicle.prediction.
                                                                trajectory.state_list,
                                                                margin=150)
        save_sce = self.sce
        self.sce_without_ego_and_other()
        self._initialize_vis(figsize=figsize, plot_limit=plot_limits)
        self.rnd.render()
        self.sce = save_sce
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step::5],
                                  color=TUMcolor.TUMlightgray, linewidth=1, start_time_step=0)
        utils_vis.draw_state_list(self.rnd, self.other_vehicle.prediction.trajectory.state_list[self.time_step::5],
                                  color=TUMcolor.TUMgreen, linewidth=1, start_time_step=0)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, time_step=self.time_step,
                                         color=TUMcolor.TUMblack, alpha=1)
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, time_step=self.time_step,
                                         color=TUMcolor.TUMgreen, alpha=1)
        if self.other_vehicle_exit_time is not None:
            utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, time_step=self.other_vehicle_exit_time,
                                             color=TUMcolor.TUMgreen)
        if self.other_vehicle_enter_time is not None:
            utils_vis.draw_dyn_vehicle_shape(self.rnd, self.other_vehicle, time_step=self.other_vehicle_enter_time,
                                             color=TUMcolor.TUMgreen)

        plt.title(f"{self.measure_name} of {self.value} time steps")
        if self.ca is not None:
            x_i, y_i = self.ca.exterior.xy
            plt.plot(x_i, y_i, color=TUMcolor.TUMblack, zorder=1001)
            plt.fill(x_i, y_i, color=TUMcolor.TUMred, zorder=1001)

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()

    def sce_without_ego_and_other(self):
        self.sce.remove_obstacle(self.ego_vehicle)
        self.sce.remove_obstacle(self.other_vehicle)

    def info_value_not_exist(self):
        if self.ca is None:
            utils_log.print_and_log_info(logger, f"* \t\tconflict area does not exist, PET is set to inf.")
        elif math.isinf(self.value):
            if self.other_vehicle_enter_time == None:
                utils_log.print_and_log_info(logger,
                                             "* \t\tThe other vehicle never encroaches the CA, PET is set to inf.")
            else:
                utils_log.print_and_log_info(logger,
                                             "* \t\tThe ego vehicle encroaches the CA, "
                                             "but never leaves it, PET is set to inf.")