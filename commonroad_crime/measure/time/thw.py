__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.4.2"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import matplotlib.pyplot as plt
import logging
import math
import numpy as np

from commonroad.geometry.shape import Rectangle, Circle
from commonroad.scenario.obstacle import Obstacle

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class THW(CriMeBase):
    """
    https://criticality-metrics.readthedocs.io/en/latest/time-scale/THW.html
    """

    measure_name = TypeTime.THW
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(THW, self).__init__(config)

    def _compute_vehicle_add_on(self, vehicle: Obstacle):
        if isinstance(vehicle.obstacle_shape, Rectangle):
            add_on = vehicle.obstacle_shape.length / 2
        elif isinstance(vehicle.obstacle_shape, Circle):
            add_on = vehicle.obstacle_shape.radius
        else:
            raise ValueError(
                f"<{self.measure_name}>: obstacle shape {type(vehicle.obstacle_shape).__name__} not supported."
            )
        return add_on

    def cal_headway(self, verbose=True):
        try:
            other_position = self.other_vehicle.state_at_time(self.time_step).position
            other_s, _ = self.clcs.convert_to_curvilinear_coords(
                other_position[0], other_position[1]
            )
        except ValueError as e:
            utils_log.print_and_log_warning(
                logger,
                f"* <THW> During the projection of the vehicle {self.other_vehicle.obstacle_id} "
                f"at time step {self.time_step}: {e}",
                verbose,
            )
            # out of projection domain: the other vehicle is far away
            return math.inf
        try:
            ego_position = self.ego_vehicle.state_at_time(self.time_step).position
            ego_s, _ = self.clcs.convert_to_curvilinear_coords(
                ego_position[0], ego_position[1]
            )
        except ValueError as e:
            utils_log.print_and_log_warning(
                logger,
                f"* <THW> During the projection of the ego vehicle with id {self.ego_vehicle.obstacle_id} "
                f"at time step {self.time_step}: {e}",
                verbose,
            )
            # out of projection domain: the ref path should be problematic
            return math.nan

        # additional position for the vehicles
        ego_s += self._compute_vehicle_add_on(self.ego_vehicle)
        other_s -= self._compute_vehicle_add_on(self.other_vehicle)

        if ego_s > other_s:
            return math.inf
        # another option is (other_s-ego_s)/self.ego_vehicle.state_at_time(self.time_step).velocity
        # here since the predicted trajectory is given, we use it to make the result more accurate
        for ts in range(
            self.time_step + 1, self.ego_vehicle.prediction.final_time_step + 1
        ):
            ego_position = self.ego_vehicle.state_at_time(ts).position
            ego_s, _ = self.clcs.convert_to_curvilinear_coords(
                ego_position[0], ego_position[1]
            )
            ego_s += self._compute_vehicle_add_on(self.ego_vehicle)
            if ego_s > other_s:
                return utils_gen.int_round(
                    (ts - self.time_step) * self.dt, str(self.dt)[::-1].find(".")
                )
        return math.inf

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        if not self.validate_update_states_log(vehicle_id, time_step, verbose):
            return np.nan

        if self._except_obstacle_in_same_lanelet(
            expected_value=math.inf, verbose=verbose
        ):
            return self.value
        self.value = self.cal_headway(verbose=verbose)
        if math.isfinite(self.value):
            self.value = utils_gen.int_round(self.value, 2)
        utils_log.print_and_log_info(
            logger,
            f"*\t\t {self.measure_name} with vehicle id {vehicle_id} = {self.value}",
            verbose,
        )
        return self.value

    def visualize(self, figsize: tuple = (25, 15)):
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
            color=TUMcolor.TUMblue,
            linewidth=5,
        )
        if self.value > 0 and self.value is not math.inf:
            tshw = int(utils_gen.int_round(self.value / self.dt, 0))
            utils_vis.draw_state(
                self.rnd, self.ego_vehicle.state_at_time(tshw + self.time_step)
            )
            utils_vis.draw_dyn_vehicle_shape(
                self.rnd, self.ego_vehicle, tshw + self.time_step
            )
        else:
            tshw = self.value
        plt.title(f"{self.measure_name} at time step {tshw}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(
                self.measure_name, self.configuration.general.path_output, tshw
            )
        else:
            plt.show()
