__author__ = "Yuanfei Lin and Kun Qian"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import logging
import matplotlib.pyplot as plt
import math
from commonroad.scenario.scenario import Tag

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeIndex, TypeMonotone
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.measure.time.pet import PET
from commonroad.scenario.obstacle import DynamicObstacle
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class CI(CriMeBase):
    """
    W. K. Alhajyaseen, “The integration of conflict probability and severity for the safety assessment of intersections”,
    Arabian Journal for Science and Engineering, vol. 40, no. 2, pp. 421–430, 2015.
    """

    measure_name = TypeIndex.CI
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(CI, self).__init__(config)
        self._pet_object = PET(config)
        self.ci_config = config.index.ci

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(
            logger,
            f"* Computing the {self.measure_name} at time step {time_step}",
            verbose,
        )
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step

        # Only for scenarios with intersection tag
        if (Tag.INTERSECTION not in self.sce.tags) or (
            len(self.sce.lanelet_network.intersections) == 0
        ):
            utils_log.print_and_log_info(
                logger,
                f"* \t\t Measure only for intersection. CI is set to 0.0.",
                verbose,
            )
            self.value = 0.0
            return self.value
        if not isinstance(self.other_vehicle, DynamicObstacle):
            utils_log.print_and_log_info(
                logger,
                f"*\t\t {self.other_vehicle} is not a dynamic obstacle, CI is set to 0.0",
                verbose,
            )
            self.value = 0.0
            return self.value

        # Alpha determines to which degree released collision energy affects vehicle occupants.
        alpha = self.ci_config.alpha
        # Beta depends on conflict type and surrounding collisions
        beta = self.ci_config.beta

        # Standard value for mass of vehicles 1500kg
        if self.configuration.vehicle.dynamic.parameters.m is not None:
            m1 = self.configuration.vehicle.dynamic.parameters.m
        else:
            m1 = self.ci_config.m
        if self.ci_config.m_b:
            m2 = self.ci_config.m_b
        else:
            # assume that the vehicle b has the same mass as the ego vehicle
            m2 = m1

        # Use PET object
        pet = self._pet_object.compute(vehicle_id, time_step)

        # Threshold of 5.0 as standard configuration in the paper
        threshold = self.ci_config.pet_threshold

        # As PET = math.inf in case of no conflict area for PET, no separate check is needed
        if pet >= threshold:
            utils_log.print_and_log_info(
                logger,
                f"*\t\t PET beyond configuration threshold, "
                f"CI for the ego vehicle and vehicle {vehicle_id} is set to 0",
                verbose,
            )
            self.value = 0.0
            return self.value

        conflict_state_ego = self.ego_vehicle.state_at_time(
            self._pet_object.ego_vehicle_enter_time
        )
        conflict_state_other = self.other_vehicle.state_at_time(
            self._pet_object.other_vehicle_enter_time
        )

        theta1 = conflict_state_ego.orientation
        theta2 = conflict_state_other.orientation

        u1 = math.sqrt(
            conflict_state_ego.velocity**2 + conflict_state_ego.velocity_y**2
        )
        u2 = math.sqrt(
            conflict_state_other.velocity**2 + conflict_state_other.velocity_y**2
        )

        # the speed after collision
        v = (m1 * u1 * math.sin(theta1) + m2 * u2 * math.sin(theta2)) / (
            (m1 + m2)
            * math.sin(
                math.atan(
                    m1 * u1 * math.sin(theta1)
                    + m2 * u2 * math.sin(theta2) / m1 * u1 * math.cos(theta1)
                    + m2 * u2 * math.cos(theta2)
                )
            )
        )

        # k_delta to calculate the kinetic energy difference before and after the collision
        k_delta = 1 / 2 * (m1 * u1**2) + 1 / 2 * (m2 * u2**2) - 1 / 2 * (m1 + m2) * v**2
        self.value = utils_gen.int_round(alpha * k_delta / math.exp(beta * pet), 2)
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}"
        )
        return self.value

    def visualize(self, figsize: tuple = (25, 15)):
        if self._pet_object.ca is None:
            return
        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(
                self.time_step,
                self.ego_vehicle.prediction.trajectory.state_list,
                margin=50,
            )
        self._initialize_vis(figsize=figsize, plot_limit=plot_limits)
        self.rnd.render()
        utils_vis.draw_state_list(
            self.rnd,
            self.ego_vehicle.prediction.trajectory.state_list[self.time_step :],
            color=TUMcolor.TUMlightgray,
            linewidth=1,
            start_time_step=0,
        )
        utils_vis.draw_state_list(
            self.rnd,
            self.other_vehicle.prediction.trajectory.state_list[self.time_step :],
            color=TUMcolor.TUMlightgray,
            linewidth=1,
            start_time_step=0,
        )
        utils_vis.draw_dyn_vehicle_shape(
            self.rnd,
            self.ego_vehicle,
            time_step=self.time_step,
            color=TUMcolor.TUMblack,
        )
        utils_vis.draw_dyn_vehicle_shape(
            self.rnd,
            self.other_vehicle,
            time_step=self.time_step,
            color=TUMcolor.TUMgreen,
        )

        plt.title(f"{self.measure_name} of {self.value} time steps")

        x_i, y_i = self._pet_object.ca.exterior.xy
        plt.plot(x_i, y_i, color=TUMcolor.TUMblack, zorder=1001)
        plt.fill(x_i, y_i, color=TUMcolor.TUMred, zorder=1001)

        state_ego = self.ego_vehicle.state_at_time(
            self._pet_object.ego_vehicle_enter_time
        )
        state_other = self.other_vehicle.state_at_time(
            self._pet_object.other_vehicle_enter_time
        )
        centroid = self._pet_object.ca.centroid
        utils_vis.draw_dyn_vehicle_shape(
            self.rnd,
            self.ego_vehicle,
            time_step=self._pet_object.ego_vehicle_enter_time,
            color=TUMcolor.TUMblack,
            alpha=0.5,
        )
        utils_vis.draw_dyn_vehicle_shape(
            self.rnd,
            self.other_vehicle,
            time_step=self._pet_object.other_vehicle_enter_time,
            color=TUMcolor.TUMgreen,
            alpha=0.5,
        )
        plt.arrow(
            x=centroid.x,
            y=centroid.y,
            dx=state_ego.velocity,
            dy=state_ego.velocity_y,
            head_width=0.4,
            width=0.1,
            ec=TUMcolor.TUMblack,
            zorder=1002,
        )
        plt.arrow(
            x=centroid.x,
            y=centroid.y,
            dx=state_other.velocity,
            dy=state_other.velocity_y,
            head_width=0.4,
            width=0.1,
            ec=TUMcolor.TUMgreen,
            zorder=1002,
        )
        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(
                    self.measure_name,
                    self.configuration.general.path_output,
                    self.time_step,
                )
            else:
                plt.show()
