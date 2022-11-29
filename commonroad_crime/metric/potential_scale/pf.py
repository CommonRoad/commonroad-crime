__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import logging

import numpy as np
from commonroad.scenario.scenario import State

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypePotentialScale
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol

logger = logging.getLogger(__name__)


class PF(CriMeBase):
    """
    Jerk is the rate of change in acceleration, and thus quantifies over the abruptness of a maneuver.
    """
    metric_name = TypePotentialScale.PF

    def __init__(self, config: CriMeConfiguration):
        super(PF, self).__init__(config)

    def compute(self,  time_step: int):
        self.time_step = time_step
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        evaluated_state = self.ego_vehicle.state_at_time(self.time_step)
        d_ego = self.clcs.convert_to_curvilinear_coords(evaluated_state.position[0],
                                                        evaluated_state.position[1])[1]
        u_total = self._calc_lane_potential(evaluated_state, d_ego) +\
            self._calc_road_potential(evaluated_state, d_ego)
        print(u_total)

    def _calc_lane_potential(self, veh_state: State, d_veh: float):
        """
        Calculates the lane potential.
        """
        def gaussian_like_function(y, y_c, sigma, A_lane):
            # Sec.II.A in Wolf, M.T. and Burdick, J.W., 2008, May. Artificial potential functions for highway
            # driving with collision avoidance. In 2008 IEEE International Conference on Robotics and
            # Automation (pp. 3731-3736). IEEE.
            return A_lane * np.exp(-(y - y_c)**2/(2 * sigma)**2)

        # we assume that the lanelet are straight after converting to the curvilinear coordinate system
        # the lanelet that the vehicle is currently occupying
        left_adj_lanelet = right_adj_lanelet = veh_lanelet = self.sce.lanelet_network.find_lanelet_by_id(
            self.sce.lanelet_network.find_lanelet_by_position([veh_state.position])[0][0])
        lanelet_list = [veh_lanelet]

        # collects all the lanelets
        while left_adj_lanelet.adj_left_same_direction:
            left_adj_lanelet = self.sce.lanelet_network.find_lanelet_by_id(left_adj_lanelet.adj_left)
            lanelet_list.append(left_adj_lanelet)
        while right_adj_lanelet.adj_right_same_direction:
            right_adj_lanelet = self.sce.lanelet_network.find_lanelet_by_id(right_adj_lanelet.adj_left)
            lanelet_list.append(right_adj_lanelet)

        # compute the lane potential
        u_lane = 0
        for ll in lanelet_list:
            # the d-coordinate of the lanelet center
            d_yc = self.clcs.convert_to_curvilinear_coords(ll.center_vertices[0][0],
                                                           ll.center_vertices[0][1])[1]
            ll_width = utils_sol.compute_lanelet_width_orientation(ll, veh_state.position)[0]
            u_lane += gaussian_like_function(d_veh, d_yc,
                                             self.configuration.potential_scale.sigma_factor * ll_width,
                                             self.configuration.potential_scale.A_lane)
        return u_lane

    def _calc_road_potential(self, veh_state: State, d_veh: float):
        """
        Calculates the road potential, which prevents the vehicle from leaving the highway
        """
        def repulsive_potential(eta: float, y: float, y_0: float):
            return 0.5 * eta * (1/(y - y_0))**2
        left_b, right_b = utils_sol.obtain_road_boundary(veh_state, self.sce.lanelet_network)
        
        # possibly: Coordinate outside of projection domain.
        # d_yb_l = self.clcs.convert_to_curvilinear_coords(left_b[0][0], left_b[0][1])[1]
        # d_yb_r = self.clcs.convert_to_curvilinear_coords(right_b[0][0], right_b[0][1])[1]
        u_road = 0
        for d_yb in [d_yb_l, d_yb_r]:
            u_road += repulsive_potential(self.configuration.potential_scale.scale_factor, d_veh, d_yb)
        return u_road

    def visualize(self):
        pass
