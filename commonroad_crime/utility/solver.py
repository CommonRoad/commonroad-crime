__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from typing import Tuple, Union
import numpy as np
import logging
from scipy.spatial.distance import cdist

from commonroad.scenario.obstacle import Obstacle, StaticObstacle, State
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.geometry.util import resample_polyline

logger = logging.getLogger(__name__)


def solver_wttc(veh_1: Obstacle,
                veh_2: Obstacle,
                time_step: int,
                a_max: float, ):
    """
    Analytical solution of the worst-time-to-collision.
    """
    r_v1, _ = compute_disc_radius_and_distance(veh_1.obstacle_shape.length,
                                               veh_1.obstacle_shape.width)
    r_v2, _ = compute_disc_radius_and_distance(veh_2.obstacle_shape.length,
                                               veh_2.obstacle_shape.width)
    x_10, y_10 = veh_1.state_at_time(time_step).position
    x_20, y_20 = veh_2.state_at_time(time_step).position
    v_1x0, v_1y0 = veh_1.state_at_time(time_step).velocity, veh_1.state_at_time(time_step).velocity_y
    v_2x0, v_2y0 = veh_2.state_at_time(time_step).velocity, veh_2.state_at_time(time_step).velocity_y
    a_10 = a_20 = a_max
    if isinstance(veh_2, StaticObstacle):
        a_20 = 0
    # compute the parameters
    A = -1 / 4 * (a_10 + a_20) ** 2
    B = 0
    C = -(a_20 + a_10) * (r_v1 + r_v2) + (v_2x0 - v_1x0) ** 2 + (v_2y0 - v_1y0) ** 2
    D = 2 * (v_2x0 - v_1x0) * (x_20 - x_10) + 2 * (v_2y0 - v_1y0) * (y_20 - y_10)
    E = (x_20 - x_10) ** 2 + (y_20 - y_10) ** 2 - (r_v1 + r_v2) ** 2
    return np.roots(np.array([A, B, C, D, E]))


def obtain_road_boundary(state: State, lanelet_network: LaneletNetwork) -> Tuple[np.ndarray, np.ndarray]:
    """
    Obtains the road boundaries based on the vehicle state.

    return: (left boundary, right boundary)
    """
    veh_lanelet_id = lanelet_network.find_lanelet_by_position([state.position])[0]
    lanelet_leftmost = lanelet_rightmost = lanelet_network.find_lanelet_by_id(veh_lanelet_id[0])
    while lanelet_leftmost.adj_left_same_direction:
        lanelet_leftmost = lanelet_network.find_lanelet_by_id(lanelet_leftmost.adj_left)
    while lanelet_rightmost.adj_right_same_direction:
        lanelet_rightmost = lanelet_network.find_lanelet_by_id(lanelet_rightmost.adj_right)
    left_bounds = resample_polyline(lanelet_leftmost.left_vertices)
    right_bounds = resample_polyline(lanelet_rightmost.right_vertices)
    return left_bounds, right_bounds


def compute_veh_dis_to_boundary(state: State, lanelet_network: LaneletNetwork) -> Tuple[float, float]:
    """
    Computes the distance between the vehicle cenver and the road boundary

    return: (distance to the right boundary,
             distance to the left boundary)
    """
    left_b, right_b = obtain_road_boundary(state, lanelet_network)
    return np.min(cdist(np.array([state.position]), right_b, "euclidean")), \
           np.min(cdist(np.array([state.position]), left_b, "euclidean"))


def compute_closest_coordinate_from_list_of_points(state: State, vertices: np.ndarray):
    """
    Computes the closest coordinate from a list of points
    """
    vertices = resample_polyline(vertices)
    return vertices[np.argmin(cdist(np.array([state.position]), vertices, "euclidean"))]


def compute_disc_radius_and_distance(length: float, width: float) \
        -> Tuple[float, float]:
    """
    Computes the radius of discs and their distances used as the approximation of the shape of the ego vehicle.
    """
    assert length >= 0 and width >= 0, f"Invalid vehicle dimensions: length = {length}, width = {width}"

    if np.isclose(length, 0.0) and np.isclose(width, 0.0):
        return 0.0, 0.0

    # half width of the ego vehicle
    half_width = width / 2

    half_length = (length / 3) / 2
    radius = (half_length ** 2 + half_width ** 2) ** 0.5

    # ceil up to 1 digit
    # radius_disc = np.ceil(radius * 10) / 10
    radius_disc = radius
    dist_circles = length / 3 * 2

    return radius_disc, dist_circles


def compute_clcs_distance(clcs: CurvilinearCoordinateSystem,
                          veh_rear_pos: np.ndarray,
                          veh_front_pos: np.ndarray) -> Tuple[float, float]:
    """
    Compute the distance between two vehicles along the curvilinear coordinate system. The sign of the distance
    is based on the assumption of the relative position of the vehicles. If the distance > 0, the relative
    position relationship holds. And vice versa.

    :param clcs: curvi-linear coordinate system
    :param veh_rear_pos: the position of the rear vehicle
    :param veh_front_pos: the position of the front vehicle

    :return the longitudinal and lateral relative distances
    """
    rear_s, rear_d = clcs.convert_to_curvilinear_coords(veh_rear_pos[0], veh_rear_pos[1])
    front_s, front_d = clcs.convert_to_curvilinear_coords(veh_front_pos[0], veh_front_pos[1])
    return front_s - rear_s, front_d - rear_d


def compute_jerk(current_acceleration: float, next_acceleration: float,
                 dt: float) -> float:
    """
    Computes jerk given acceleration

    :param current_acceleration: acceleration of current time step
    :param next_acceleration: acceleration of previous time step
    :param dt: time step size
    :return: jerk
    """
    jerk = (next_acceleration - current_acceleration) / dt
    return jerk


def compute_acceleration(current_velocity: float, next_velocity: float,
                         dt: float):
    """
    Computes acceleration given velocity

    :param current_velocity: velocity of current time step
    :param next_velocity: velocity of previous time step
    :param dt: time step size
    :return: acceleration
    """
    acceleration = (next_velocity - current_velocity) / dt
    return acceleration


def compute_lanelet_width_orientation(lanelet: Lanelet, position: np.ndarray) -> Tuple[Union[float, None],
                                                                                       Union[float, None]]:
    """
    Computes the width and the orientation of the lanelet at given position

    :param lanelet: a lanelet
    :param position: position of the vehicle
    """
    width_list = _compute_width_from_lanalet_boundary(lanelet.left_vertices, lanelet.right_vertices)
    orient_list = _compute_orientation_from_polyline(lanelet.center_vertices)

    path_length = _compute_path_length_from_polyline(lanelet.center_vertices)
    lanelet_clcs = CurvilinearCoordinateSystem(lanelet.center_vertices)
    position_s, _ = lanelet_clcs.convert_to_curvilinear_coords(position[0], position[1])
    return np.interp(position_s, path_length, width_list), np.interp(position_s, path_length, orient_list)


def _compute_width_from_lanalet_boundary(
        left_polyline: np.ndarray, right_polyline: np.ndarray
) -> np.ndarray:
    """
    Computes the width of a lanelet. Credit: Sebastian Maierhofer.

    :param left_polyline: left boundary of lanelet
    :param right_polyline: right boundary of lanelet
    :return: width along lanelet
    """
    width_along_lanelet = np.zeros((len(left_polyline),))
    for i in range(len(left_polyline)):
        width_along_lanelet[i] = np.linalg.norm(
            left_polyline[i] - right_polyline[i]
        )
    return width_along_lanelet


def _compute_path_length_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the path length of a polyline. Credit: Sebastian Maierhofer.

    :param polyline: polyline for which path length should be calculated
    :return: path length along polyline
    """
    assert (
            isinstance(polyline, np.ndarray)
            and polyline.ndim == 2
            and len(polyline[:, 0]) > 2
    ), "Polyline malformed for pathlenth computation p={}".format(polyline)

    distance = np.zeros((len(polyline),))
    for i in range(1, len(polyline)):
        distance[i] = distance[i - 1] + np.linalg.norm(
            polyline[i] - polyline[i - 1]
        )

    return np.array(distance)


def _compute_orientation_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """
    Computes orientation along a polyline. Credit: Sebastian Maierhofer.

    :param polyline: polyline for which orientation should be calculated
    :return: orientation along polyline
    """
    assert (
            isinstance(polyline, np.ndarray)
            and len(polyline) > 1
            and polyline.ndim == 2
            and len(polyline[0, :]) == 2
    ), "<Math>: not a valid polyline. polyline = {}".format(polyline)
    if len(polyline) < 2:
        raise ValueError("Cannot create orientation from polyline of length < 2")

    orientation = [0]
    for i in range(1, len(polyline)):
        pt1 = polyline[i - 1]
        pt2 = polyline[i]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    return np.array(orientation)

