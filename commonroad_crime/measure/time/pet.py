__author__ = "Yuanfei Lin, Liguo Chen"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import matplotlib.pyplot as plt
import logging
from typing import Union
import math

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
from commonroad_crime.measure.time.et import ET
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor
import commonroad_crime.utility.general as utils_gen
from commonroad.scenario.scenario import Tag
from commonroad.scenario.obstacle import DynamicObstacle

logger = logging.getLogger(__name__)


class PET(ET):
    """
    See https://criticality-metrics.readthedocs.io/. The definition is mainly obtained from B. L. Allen, B. T. Shin,
    and P. J. Cooper, “Analysis of Traffic Conflicts and Collisions,” Transportation Research Record, vol. 667,
    pp. 67–74, 1978.
    """

    measure_name = TypeTime.PET
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(PET, self).__init__(config)
        self.ca = None  # ca stands for conflict area
        self.other_vehicle_exit_time: Union[float, int] = math.inf
        self.other_vehicle_enter_time: Union[float, int] = math.inf
        self.ego_vehicle_exit_time: Union[float, int] = math.inf
        self.ego_vehicle_enter_time: Union[float, int] = math.inf
        self.case_one: bool = False

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(
            logger,
            f"* Computing the {self.measure_name} beginning at time step {time_step}",
            verbose,
        )
        self.time_step = time_step
        self.set_other_vehicles(vehicle_id)
        if (Tag.INTERSECTION not in self.sce.tags) or (
            len(self.sce.lanelet_network.intersections) == 0
        ):
            utils_log.print_and_log_info(
                logger,
                f"*\t\t There is no intersection. \n*\t\t {self.measure_name} = {self.value}",
                verbose,
            )
            self.value = math.inf
            return self.value
        if not isinstance(self.other_vehicle, DynamicObstacle):
            utils_log.print_and_log_info(
                logger,
                f"*\t\t Vehicle {self.other_vehicle} is not a dynamic obstacle",
                verbose,
            )
            utils_log.print_and_log_info(
                logger, f"*\t\t\t\t {self.measure_name} = {self.value}", verbose
            )
            self.value = math.inf
            return self.value
        # Create an agent object of ET to obtain the conflict area
        self.ca = self.get_ca(self.time_step, self.other_vehicle)
        utils_log.print_and_log_info(
            logger,
            f"*\t\t There is a conflict area in the scenario",
            verbose,
        )
        (
            _,
            self.other_vehicle_enter_time,
            self.other_vehicle_exit_time,
        ) = self.get_ca_time_info(self.other_vehicle, self.time_step, self.ca)
        (
            _,
            self.ego_vehicle_enter_time,
            self.ego_vehicle_exit_time,
        ) = self.get_ca_time_info(self.ego_vehicle, self.time_step, self.ca)
        # Definition of PET is:
        # PET(A1, A2, Conflict Area) = t_entry(A2, Conflict Area) - t_exit(A1, Conflict Area)
        # The variable A1 in the equation may not necessarily correspond to the ego vehicle.
        # Depending on the relative time points of both vehicles entering and leaving the conflict area,
        # A1 could represent either the ego vehicle or the other vehicle. So there are two cases:
        # Case 1: A1 is ego vehicle
        case_one = True
        if (
            self.other_vehicle_enter_time is math.inf
            or self.ego_vehicle_exit_time is math.inf
        ):
            self.value = math.inf
        else:
            self.value = self.other_vehicle_enter_time - self.ego_vehicle_exit_time
        # case 2: A1 is the other vehicle
        if self.value < 0:
            case_one = False
            if (
                self.ego_vehicle_enter_time is math.inf
                or self.other_vehicle_exit_time is math.inf
            ):
                self.value = math.inf
            else:
                self.value = self.ego_vehicle_enter_time - self.other_vehicle_exit_time
        if self.value < 0:
            self.value = math.inf
        # The conflict area may not exist, indicated by self.ca being None.
        # Even if the conflict area exists, there are two scenarios where the PET remains undefined,
        # and we set it to infinity. This information is logged in info_value_not_exit()
        self.case_one = case_one
        self.print_info_value_not_exist(case_one)
        # Transfer time steps to seconds
        if self.value is not math.inf:
            self.value = utils_gen.int_round(self.value * self.dt, 4)
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}", verbose
        )
        return self.value

    def visualize(self, figsize: tuple = (25, 15), verbose: bool = True):
        if self.ca is None:
            return
        if (
            self.other_vehicle_exit_time is math.inf
            and self.other_vehicle_enter_time is math.inf
        ):
            return
        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(
                self.time_step,
                self.ego_vehicle.prediction.trajectory.state_list,
                margin=150,
            )
        save_sce = self.sce
        self.sce_without_ego_and_other()
        self._initialize_vis(figsize=figsize, plot_limit=plot_limits)
        self.rnd.render()
        self.sce = save_sce
        utils_vis.draw_state_list(
            self.rnd,
            self.ego_vehicle.prediction.trajectory.state_list[self.time_step :: 5],
            color=TUMcolor.TUMlightgray,
            linewidth=1,
            start_time_step=0,
        )
        utils_vis.draw_state_list(
            self.rnd,
            self.other_vehicle.prediction.trajectory.state_list[self.time_step :: 5],
            color=TUMcolor.TUMgreen,
            linewidth=1,
            start_time_step=0,
        )
        utils_vis.draw_dyn_vehicle_shape(
            self.rnd,
            self.ego_vehicle,
            time_step=self.time_step,
            color=TUMcolor.TUMblack,
            alpha=1,
        )
        utils_vis.draw_dyn_vehicle_shape(
            self.rnd,
            self.other_vehicle,
            time_step=self.time_step,
            color=TUMcolor.TUMgreen,
            alpha=1,
        )
        if self.case_one is True:
            if self.ego_vehicle_exit_time is not math.inf:
                utils_vis.draw_dyn_vehicle_shape(
                    self.rnd,
                    self.ego_vehicle,
                    time_step=self.ego_vehicle_exit_time,
                    color=TUMcolor.TUMblack,
                )
            if self.other_vehicle_enter_time is not math.inf:
                utils_vis.draw_dyn_vehicle_shape(
                    self.rnd,
                    self.other_vehicle,
                    time_step=self.other_vehicle_enter_time,
                    color=TUMcolor.TUMgreen,
                )
        elif self.case_one is False:
            if self.other_vehicle_exit_time is not math.inf:
                utils_vis.draw_dyn_vehicle_shape(
                    self.rnd,
                    self.other_vehicle,
                    time_step=self.other_vehicle_exit_time,
                    color=TUMcolor.TUMgreen,
                )
            if self.ego_vehicle_enter_time is not math.inf:
                utils_vis.draw_dyn_vehicle_shape(
                    self.rnd,
                    self.ego_vehicle,
                    time_step=self.ego_vehicle_enter_time,
                    color=TUMcolor.TUMblack,
                )

        plt.title(f"{self.measure_name} at time step {self.time_step}")
        if self.ca is not None:
            x_i, y_i = self.ca.exterior.xy
            plt.plot(x_i, y_i, color=TUMcolor.TUMblack, zorder=1001)
            plt.fill(x_i, y_i, color=TUMcolor.TUMred, zorder=1001)

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(
                    self.measure_name,
                    self.configuration.general.path_output,
                    self.time_step,
                )
            else:
                plt.show()

    def sce_without_ego_and_other(self):
        self.sce.remove_obstacle(self.ego_vehicle)
        self.sce.remove_obstacle(self.other_vehicle)

    def print_info_value_not_exist(self, case_one):
        if self.ca is None:
            utils_log.print_and_log_info(
                logger, f"* \t\t The conflict area does not exist, PET is set to inf."
            )
            return
        if case_one is True:
            if math.isinf(self.value):
                if self.other_vehicle_enter_time is math.inf:
                    utils_log.print_and_log_info(
                        logger,
                        "*\t\t The other vehicle never encroaches the conflict area, PET is set to inf.",
                    )
                elif self.ego_vehicle_exit_time is math.inf:
                    utils_log.print_and_log_info(
                        logger,
                        "*\t\t The ego vehicle entered the conflict area, "
                        "but never leaves it, PET is set to inf.",
                    )
                elif self.other_vehicle_enter_time < self.ego_vehicle_exit_time:
                    utils_log.print_and_log_info(
                        logger,
                        "*\t\t The ego vehicle left the conflict area, "
                        "after the other vehicle entered.",
                    )
            return
        else:
            if math.isinf(self.value):
                if self.ego_vehicle_enter_time is math.inf:
                    utils_log.print_and_log_info(
                        logger,
                        "*\t\t The ego vehicle never encroaches the conflict area, PET is set to inf.",
                    )
                elif self.other_vehicle_exit_time is math.inf:
                    utils_log.print_and_log_info(
                        logger,
                        "*\t\t The other vehicle has entered the conflict area, "
                        "but never leaves it, PET is set to inf.",
                    )
                elif self.ego_vehicle_enter_time < self.other_vehicle_exit_time:
                    utils_log.print_and_log_info(
                        logger,
                        "*\t\t The other vehicle left the conflict area, "
                        "after the ego vehicle entered.",
                    )
            return
