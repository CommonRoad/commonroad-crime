__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import logging
from shapely.geometry import Polygon, Point
import numpy as np

from commonroad.scenario.scenario import State
from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle

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
        s_ego, d_ego = self.clcs.convert_to_curvilinear_coords(evaluated_state.position[0],
                                                               evaluated_state.position[1])
        u_total = self._calc_lane_potential(evaluated_state, d_ego) +\
            self._calc_road_potential(evaluated_state, d_ego) +\
            self._calc_car_potential(s_ego, d_ego, time_step)
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
        # assme that all the lanelets have the same width
        ll_width = utils_sol.compute_lanelet_width_orientation(veh_lanelet, veh_state.position)[0]

        vertices_list = []
        if left_adj_lanelet.adj_left_same_direction:
            vertices_list.append(left_adj_lanelet.left_vertices)
        if right_adj_lanelet.adj_right_same_direction:
            vertices_list.append(right_adj_lanelet.right_vertices)
        # collects all the lanelets
        while left_adj_lanelet.adj_left_same_direction:
            left_adj_lanelet = self.sce.lanelet_network.find_lanelet_by_id(left_adj_lanelet.adj_left)
            if left_adj_lanelet.adj_left_same_direction:
                vertices_list.append(left_adj_lanelet.left_vertices)
        while right_adj_lanelet.adj_right_same_direction:
            right_adj_lanelet = self.sce.lanelet_network.find_lanelet_by_id(right_adj_lanelet.adj_left)
            if right_adj_lanelet.adj_right_same_direction:
                vertices_list.append(right_adj_lanelet.right_vertices)

        # compute the lane potential
        u_lane = 0.
        for vts in vertices_list:
            closest_vts = utils_sol.compute_closest_coordinate_from_list_of_points(veh_state, vts)
            # the d-coordinate of the lanelet bounds
            d_yc = self.clcs.convert_to_curvilinear_coords(closest_vts[0], closest_vts[1])[1]
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

        # possibly: Coordinate outside of projection domain.
        # left_b, right_b = utils_sol.obtain_road_boundary(veh_state, self.sce.lanelet_network)
        # d_yb_l = self.clcs.convert_to_curvilinear_coords(left_b[0][0], left_b[0][1])[1]
        # d_yb_r = self.clcs.convert_to_curvilinear_coords(right_b[0][0], right_b[0][1])[1]

        dis_right, dis_left = utils_sol.compute_veh_dis_to_boundary(veh_state, self.sce.lanelet_network)

        u_road = 0.
        for d_yb in [d_veh - dis_right, d_veh + dis_left]:
            u_road += repulsive_potential(self.configuration.potential_scale.scale_factor, d_veh, d_yb)
        return u_road

    def _calc_car_potential(self, s_veh: float, d_veh: float, time_step: int):
        for obs in self.sce.obstacles:
            # shape in curvilinear coordinate system
            obs_clcs_shape = self.clcs.convert_list_of_polygons_to_curvilinear_coords_and_rasterize(
                [obs.occupancy_at_time(time_step).shape.shapely_object.exterior.coords], [0], 1, 4
            )[0]
            obs_clcs_poly = Polygon(obs_clcs_shape[0][0])
            obs_s_min = np.min(obs_clcs_poly.exterior.xy[0])
            if isinstance(obs, StaticObstacle) or (isinstance(obs, StaticObstacle) and s_veh > obs_s_min):
                # static obstacle or forward/side of obstacle -> Euclidean distance to the nearest point on the obstacle
                K = Point(s_veh, d_veh).distance(obs_clcs_poly)
                print(K)
            elif isinstance(obs, DynamicObstacle):
                pass
            else:
                pass
        return 0



    def visualize(self):
        pass
