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
from commonroad_crime.utility.visualization import TUMcolor


logger = logging.getLogger(__name__)


class MSD(CriMeBase):
    """
    The definition is obtained from: B. L. Allen, B. T. Shin, and P. J. Cooper, “Analysis of Traffic Conflicts and
    Collisions,” Transportation Research Record, vol. 667, pp. 67–74, 1978.
    """

    measure_name = TypeDistance.MSD
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(MSD, self).__init__(config)

    def compute(self, vehicle_id: int = None, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(
            logger,
            f"* Computing the {self.measure_name} at time step {time_step}",
            verbose,
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
        v_ego = math.sqrt(state.velocity**2 + state.velocity_y**2) * math.cos(
            ego_orientation
        )

        # compute MSD
        self.value = utils_gen.int_round(
            v_ego**2 / (2 * np.abs(self.configuration.vehicle.curvilinear.a_lon_max)),
            2,
        )

        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}", verbose
        )
        return self.value

    def compute_msd_location_time_step(self, msd: float = 0):
        """
        Computes the estimated stop position according to MSD
        """
        clcs = self.configuration.vehicle.curvilinear.clcs
        current_s, current_d = clcs.convert_to_curvilinear_coords(
            self.ego_vehicle.state_at_time(self.time_step).position[0],
            self.ego_vehicle.state_at_time(self.time_step).position[1],
        )
        msd_s = current_s + msd
        msd_x, mds_y = clcs.convert_to_cartesian_coords(msd_s, current_d)
        return np.asarray([msd_x, mds_y])

    def visualize(self, figsize: tuple = (25, 15)):
        msd_location = self.compute_msd_location_time_step(self.value)

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
        utils_vis.draw_circle(self.rnd, msd_location, 0.5, 1, color=TUMcolor.TUMred)
        plt.title(f"{self.measure_name} of {self.time_step} time steps")

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(
                    self.measure_name,
                    self.configuration.general.path_output,
                    self.time_step,
                )
            else:
                plt.show()
