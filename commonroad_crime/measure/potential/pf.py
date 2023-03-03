__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging

from shapely.geometry import Polygon, Point, LineString
import numpy as np
import matplotlib.pyplot as plt

from commonroad.scenario.state import State
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypePotential, TypeMonotone
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.visualization as utils_vis

logger = logging.getLogger(__name__)


class PF(CriMeBase):
    """
    Jerk is the rate of change in acceleration, and thus quantifies over the abruptness of a maneuver.
    """
    measure_name = TypePotential.PF
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(PF, self).__init__(config)
        self._s_ego = None
        self._d_egp = None

    def compute(self, time_step: int, vehicle_id: int=None):
        self.time_step = time_step
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        evaluated_state = self.ego_vehicle.state_at_time(self.time_step)
        try:
            self._s_ego, self._d_ego = self.clcs.convert_to_curvilinear_coords(evaluated_state.position[0],
                                                                               evaluated_state.position[1])
        except ValueError as err:
            utils_log.print_and_log_error(logger, err)
            return None
        self.value = self.calc_total_potential(evaluated_state, self._s_ego, self._d_ego)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def calc_total_potential(self, veh_state: State, s_veh: float, d_veh: float):
        u_total = self._calc_lane_potential(veh_state, d_veh) + \
                  self._calc_road_potential(veh_state, d_veh) + \
                  self._calc_car_potential(veh_state, s_veh, d_veh)
        if self.configuration.potential.desired_speed:
            u_total += self._calc_velocity_potential(veh_state, s_veh)
        if u_total == np.inf or u_total >= self.configuration.potential.u_max:
            return self.configuration.potential.u_max
        else:
            return utils_gen.int_round(u_total, 2)

    def _calc_lane_potential(self, veh_state: State, d_veh: float):
        """
        Calculates the lane potential.
        """

        def gaussian_like_function(y, y_c, sigma, A_lane):
            # Sec.II.A in Wolf, M.T. and Burdick, J.W., 2008, May. Artificial potential functions for highway
            # driving with collision avoidance. In 2008 IEEE International Conference on Robotics and
            # Automation (pp. 3731-3736). IEEE.
            return A_lane * np.exp(-(y - y_c) ** 2 / (2 * sigma) ** 2)

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
            right_adj_lanelet = self.sce.lanelet_network.find_lanelet_by_id(right_adj_lanelet.adj_right)
            if right_adj_lanelet.adj_right_same_direction:
                vertices_list.append(right_adj_lanelet.right_vertices)

        # compute the lane potential
        u_lane = 0.
        for vts in vertices_list:
            closest_vts = utils_sol.compute_closest_coordinate_from_list_of_points(veh_state, vts)
            # the d-coordinate of the lanelet bounds
            d_yc = self.clcs.convert_to_curvilinear_coords(closest_vts[0], closest_vts[1])[1]
            u_lane += gaussian_like_function(d_veh, d_yc,
                                             self.configuration.potential.sigma_factor * ll_width,
                                             self.configuration.potential.A_lane)
        return u_lane

    def _calc_road_potential(self, veh_state: State, d_veh: float):
        """
        Calculates the road potential, which prevents the vehicle from leaving the highway
        """

        def repulsive_potential(eta: float, y: float, y_0: float):
            return 0.5 * eta * (1 / (y - y_0)) ** 2

        # possibly: Coordinate outside of projection domain.
        # left_b, right_b = utils_sol.obtain_road_boundary(veh_state, self.sce.lanelet_network)
        # d_yb_l = self.clcs.convert_to_curvilinear_coords(left_b[0][0], left_b[0][1])[1]
        # d_yb_r = self.clcs.convert_to_curvilinear_coords(right_b[0][0], right_b[0][1])[1]

        dis_right, dis_left = utils_sol.compute_veh_dis_to_boundary(veh_state, self.sce.lanelet_network)

        u_road = 0.
        for d_yb in [self._d_ego + dis_left, self._d_ego - dis_right]:
            u_road += repulsive_potential(self.configuration.potential.scale_factor, d_veh, d_yb)

        return u_road

    def _calc_car_potential(self, veh_state: State, s_veh: float, d_veh: float):
        def calc_scale_factor(d_0, v, T_f, beta, v_m):
            if v >= d_0 / T_f:
                xi_0 = d_0 / (T_f * v)
            else:
                xi_0 = 1
            xi_m = xi_0 * np.exp(-beta * (v - v_m))
            return xi_m

        config_pot = self.configuration.potential
        u_car = 0
        for obs in self.sce.obstacles:
            if obs is not self.ego_vehicle:
                # shape in curvilinear coordinate system
                obs_clcs_shape = self.clcs.convert_list_of_polygons_to_curvilinear_coords_and_rasterize(
                    [obs.occupancy_at_time(self.time_step).shape.shapely_object.exterior.coords], [0], 1, 4
                )[0]
                if len(obs_clcs_shape[0]) == 0:
                    utils_log.print_and_log_warning(
                        logger, f"At Time step {self.time_step}: the conversion of the polygon to the "
                                f"curvilinear coordinates failed, u_car is set to 0"
                    )
                    u_car += 0
                    continue
                obs_clcs_poly = Polygon(obs_clcs_shape[0][0])
                obs_s_min = np.min(obs_clcs_poly.exterior.xy[0])
                if isinstance(obs, StaticObstacle) or (isinstance(obs, DynamicObstacle)
                                                       and (s_veh > obs_s_min).any()):
                    # static obstacle or forward/side of obstacle ->
                    # Euclidean distance to the nearest point on the obstacle
                    if obs_clcs_poly.contains(Point(s_veh, d_veh)):
                        K = 0.
                    else:
                        K = Point(s_veh, d_veh).distance(obs_clcs_poly)
                else:
                    # behind dynamic obstacle
                    minx, miny = np.array(obs_clcs_shape[0][0]).min(axis=0)
                    maxx, maxy = np.array(obs_clcs_shape[0][0]).max(axis=0)
                    bottommost_then_leftmost_point = np.array([minx, maxy])
                    topmost_then_leftmost_point = np.array([minx, miny])
                    # * previous option: the sort of the orders but doesn't work for some scenarios
                    # topmost_then_leftmost_point = min(obs_clcs_shape[0][0], key=lambda pt: (-pt[0], pt[1]))
                    # bottommost_then_leftmost_point = min(obs_clcs_shape[0][0], key=lambda pt: (-pt[1], pt[0]))
                    wedge_point_l = LineString(
                        [topmost_then_leftmost_point,
                         (topmost_then_leftmost_point + bottommost_then_leftmost_point) / 2]).parallel_offset(
                        abs(self.configuration.potential.wedge_vertex), 'left'
                    ).boundary[1]
                    wedge = Polygon([wedge_point_l,
                                     topmost_then_leftmost_point, bottommost_then_leftmost_point])

                    obs_with_wedge = obs_clcs_poly.union(wedge)
                    # scaled s-coordinate
                    scale = calc_scale_factor(config_pot.d_0, veh_state.velocity, config_pot.follow_time,
                                              config_pot.beta, obs.state_at_time(self.time_step).velocity)
                    s_veh_scaled = scale * (s_veh - obs_s_min) + obs_s_min
                    if obs_with_wedge.contains(Point(s_veh_scaled, d_veh)):
                        K = 0.
                    else:
                        K = Point(s_veh_scaled, d_veh).distance(obs_with_wedge)
                u_car += config_pot.A_car * np.exp(- config_pot.alpha * K) / (K + 10e-6)
        return u_car

    def _calc_velocity_potential(self, veh_state: State, s_veh: float):
        return self.configuration.potential.slope_scale * (
                veh_state.velocity - self.configuration.potential.desired_speed
        ) * s_veh

    def visualize(self, figsize: tuple = (25, 15)):
        dis_right, dis_left = utils_sol.compute_veh_dis_to_boundary(self.ego_vehicle.state_at_time(self.time_step),
                                                                    self.sce.lanelet_network)
        d_bounds = [self._d_ego - dis_right, self._d_ego + dis_left]
        s = np.linspace(self._s_ego - 15, self._s_ego + 55, 50)
        d = np.linspace(d_bounds[0]-0.5, d_bounds[1]+0.5, 50)
        S, D = np.meshgrid(s, d)
        U = np.zeros((len(s), len(d)))
        evaluated_state = self.ego_vehicle.state_at_time(self.time_step)

        for i in range(len(s)):
            for j in range(len(d)):
                U[i, j] = self.calc_total_potential(evaluated_state, S[i, j], D[i, j])

        # polygons
        for obs in self.sce.obstacles:
            if obs is not self.ego_vehicle:
                # shape in curvilinear coordinate system
                obs_clcs_shape = self.clcs.convert_list_of_polygons_to_curvilinear_coords_and_rasterize(
                    [obs.occupancy_at_time(self.time_step).shape.shapely_object.exterior.coords], [0], 1, 4
                )[0]
                if len(obs_clcs_shape[0]) == 0:
                    utils_log.print_and_log_warning(
                        logger, f"At Time step {self.time_step}: the conversion of the polygon to the "
                                f"curvilinear coordinates failed, u_car is set to 0"
                    )
                    continue
                obs_clcs_poly = Polygon(obs_clcs_shape[0][0])
                if isinstance(obs, StaticObstacle):
                    plt.plot(*obs_clcs_poly.exterior.xy)
                else:
                    minx, miny = np.array(obs_clcs_shape[0][0]).min(axis=0)
                    maxx, maxy = np.array(obs_clcs_shape[0][0]).max(axis=0)
                    bottommost_then_leftmost_point = np.array([minx, maxy])
                    topmost_then_leftmost_point = np.array([minx, miny])
                    # * previous option: the sort of the orders but doesn't work for some scenarios
                    # topmost_then_leftmost_point = min(obs_clcs_shape[0][0], key=lambda pt: (pt[0], pt[1]))
                    # bottommost_then_leftmost_point = min(obs_clcs_shape[0][0], key=lambda pt: (-pt[1], pt[0]))
                    wedge_point_l = LineString(
                        [topmost_then_leftmost_point,
                         (topmost_then_leftmost_point + bottommost_then_leftmost_point) / 2]).parallel_offset(
                        abs(self.configuration.potential.wedge_vertex), 'left'
                    ).boundary[1]
                    wedge = Polygon([wedge_point_l,
                                     topmost_then_leftmost_point, bottommost_then_leftmost_point])
                    obs_with_wedge = obs_clcs_poly.union(wedge)
                    plt.plot(*obs_clcs_poly.exterior.xy)
                    plt.plot(*wedge.exterior.xy)
                    #plt.plot(*obs_with_wedge.exterior.xy)
        plt.contour(S, D, U, 20, cmap='RdBu_r')
        plt.colorbar()
        # lane boundaries
        plt.plot([self._s_ego - 15, self._s_ego + 55],
                 [d_bounds[0], d_bounds[0]], 'k', linewidth=3)
        plt.plot([self._s_ego - 15, self._s_ego + 55],
                 [d_bounds[1], d_bounds[1]], 'k', linewidth=3)
        plt.axis('equal')
        plt.title(f"{self.measure_name} at time step {self.time_step} is {self.value}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()
