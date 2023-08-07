__author__ = "Yuanfei Lin, Ziqian Xu"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import math
import matplotlib.pyplot as plt
import logging

import numpy as np

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeDistance, TypeMonotone
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.visualization as utils_vis
from commonroad.geometry.polyline_util import (
    compute_polyline_lengths,
    compute_polyline_intersections,
    is_point_on_polyline,
    compute_total_polyline_length,
)
from commonroad_crime.utility.visualization import TUMcolor


logger = logging.getLogger(__name__)


class MSD(CriMeBase):
    """
    B. L. Allen, B. T. Shin, and P. J. Cooper, “Analysis of Traffic Conflicts and Collisions,” Transportation Research Record, vol. 667, pp. 67–74, 1978.
    """

    measure_name = TypeDistance.MSD
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(MSD, self).__init__(config)

    def compute(self, vehicle_id: int = None, time_step: int = 0):
        utils_log.print_and_log_info(
            logger, f"* Computing the {self.measure_name} at time step {time_step}"
        )
        self.time_step = time_step
        state = self.ego_vehicle.state_at_time(time_step)
        lanelet_id = self.sce.lanelet_network.find_lanelet_by_position(
            [self.ego_vehicle.state_at_time(time_step).position]
        )[0]

        # compute the orientation of ego-vehicle
        ego_orientation = utils_sol.compute_lanelet_width_orientation(
            self.sce.lanelet_network.find_lanelet_by_id(lanelet_id[0]),
            self.ego_vehicle.state_at_time(time_step).position,
        )[1]

        # actual velocity and acceleration of ego-vehicle along the lanelet
        v_ego = (
            np.sign(state.velocity)
            * math.sqrt(state.velocity**2 + state.velocity_y**2)
            * math.cos(ego_orientation)
        )
        a_ego = (
            np.sign(state.acceleration)
            * math.sqrt(state.acceleration**2 + state.acceleration_y**2)
            * math.cos(ego_orientation)
        )

        # compute MSD
        if a_ego == 0.0:
            self.value = math.inf
        else:
            self.value = utils_gen.int_round(v_ego**2 / (2 * np.abs(a_ego)), 2)

        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}"
        )
        return self.value

    def MSD_position(self, msd: float = 0, time_step: int = 0):
        # compute the estimated stop position according to MSD
        distance = 0
        state_list = self.ego_vehicle.prediction.trajectory.state_list[time_step:]
        for ts in range(time_step + 2, self.ego_vehicle.prediction.final_time_step + 1):
            state_list = self.ego_vehicle.prediction.trajectory.state_list[time_step:ts]
            pos = np.asarray([state.position for state in state_list])
            distance = compute_total_polyline_length(pos)
            if distance > msd or distance == msd:
                msd_position = self.ego_vehicle.state_at_time(ts - 1).position
                self.msd_timestep = ts - 1
                return msd_position

    def visualize(self, figsize: tuple = (25, 15)):
        msd_position = self.MSD_position(self.value, self.time_step)

        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(
                self.time_step,
                self.ego_vehicle.prediction.trajectory.state_list,
                margin=10,
            )

        if self.value == math.inf:
            utils_log.print_and_log_info(logger, "* msd is infinity")

        elif self.value == 0:
            utils_log.print_and_log_info(logger, "* msd is zero")

        else:
            self._initialize_vis(figsize=figsize, plot_limit=plot_limits)
            self.rnd.render()
            utils_vis.draw_state_list(
                self.rnd,
                self.ego_vehicle.prediction.trajectory.state_list[self.time_step :],
                color=TUMcolor.TUMblue,
                linewidth=1,
            )
            utils_vis.draw_dyn_vehicle_shape(
                self.rnd,
                self.ego_vehicle,
                time_step=self.time_step,
                color=TUMcolor.TUMgreen,
            )
            utils_vis.draw_dyn_vehicle_shape(
                self.rnd,
                self.ego_vehicle,
                time_step=self.msd_timestep,
                color=TUMcolor.TUMorange,
            )
            utils_vis.draw_circle(self.rnd, msd_position, 1, 0.5, color=TUMcolor.TUMred)
            plt.title(f"{self.metric_name} of {self.time_step} time steps")

            if self.configuration.debug.draw_visualization:
                if self.configuration.debug.save_plots:
                    utils_vis.save_fig(
                        self.metric_name,
                        self.configuration.general.path_output,
                        self.time_step,
                    )
                else:
                    plt.show()
