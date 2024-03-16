__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.2"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from typing import Tuple, Union
import numpy as np
import logging
import math
from shapely.geometry import Polygon
from scipy.spatial.distance import cdist

from commonroad.scenario.obstacle import (
    Obstacle,
    StaticObstacle,
    ObstacleType,
    DynamicObstacle,
)
from commonroad.scenario.state import State
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.geometry.util import (
    resample_polyline,
    compute_orientation_from_polyline,
    compute_pathlength_from_polyline,
)
import commonroad_dc.pycrccosy as pycrccosy

from scipy.interpolate import splprep, splev

logger = logging.getLogger(__name__)


def solver_wttc(
    veh_1: Obstacle,
    veh_2: Obstacle,
    time_step: int,
    a_max: float,
):
    """
    Analytical solution of the worst-time-to-collision.
    """
    if veh_1.obstacle_type == ObstacleType.PEDESTRIAN:
        r_v1 = veh_1.obstacle_shape.radius
    else:
        r_v1, _ = compute_disc_radius_and_distance(
            veh_1.obstacle_shape.length, veh_1.obstacle_shape.width
        )
    if veh_2.obstacle_type == ObstacleType.PEDESTRIAN:
        r_v2 = veh_2.obstacle_shape.radius
    else:
        r_v2, _ = compute_disc_radius_and_distance(
            veh_2.obstacle_shape.length, veh_2.obstacle_shape.width
        )
    x_10, y_10 = veh_1.state_at_time(time_step).position
    x_20, y_20 = veh_2.state_at_time(time_step).position
    v_1x0, v_1y0 = (
        veh_1.state_at_time(time_step).velocity,
        veh_1.state_at_time(time_step).velocity_y,
    )
    v_2x0, v_2y0 = (
        veh_2.state_at_time(time_step).velocity,
        veh_2.state_at_time(time_step).velocity_y,
    )
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


def obtain_road_boundary(
    state: State, lanelet_network: LaneletNetwork
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Obtains the road boundaries based on the vehicle state.

    :return: (left boundary, right boundary)
    """
    veh_lanelet_id = lanelet_network.find_lanelet_by_position([state.position])[0]
    lanelet_leftmost = lanelet_rightmost = lanelet_network.find_lanelet_by_id(
        veh_lanelet_id[0]
    )
    while lanelet_leftmost.adj_left_same_direction:
        lanelet_leftmost = lanelet_network.find_lanelet_by_id(lanelet_leftmost.adj_left)
    while lanelet_rightmost.adj_right_same_direction:
        lanelet_rightmost = lanelet_network.find_lanelet_by_id(
            lanelet_rightmost.adj_right
        )
    left_bounds = resample_polyline(lanelet_leftmost.left_vertices)
    right_bounds = resample_polyline(lanelet_rightmost.right_vertices)
    return left_bounds, right_bounds


def compute_veh_dis_to_boundary(
    state: State, lanelet_network: LaneletNetwork
) -> Tuple[float, float]:
    """
    Computes the distance between the vehicle cenver and the road boundary

    :return: (distance to the right boundary,
             distance to the left boundary)
    """
    left_b, right_b = obtain_road_boundary(state, lanelet_network)
    return np.min(cdist(np.array([state.position]), right_b, "euclidean")), np.min(
        cdist(np.array([state.position]), left_b, "euclidean")
    )


def compute_closest_coordinate_from_list_of_points(state: State, vertices: np.ndarray):
    """
    Computes the closest coordinate from a list of points
    """
    vertices = resample_polyline(vertices)
    return vertices[np.argmin(cdist(np.array([state.position]), vertices, "euclidean"))]


def compute_disc_radius_and_distance(
    length: float, width: float
) -> Tuple[float, float]:
    """
    Computes the radius of discs and their distances used as the approximation of the shape of the ego vehicle.
    """
    assert (
        length >= 0 and width >= 0
    ), f"Invalid vehicle dimensions: length = {length}, width = {width}"

    if np.isclose(length, 0.0) and np.isclose(width, 0.0):
        return 0.0, 0.0

    # half width of the ego vehicle
    half_width = width / 2

    half_length = (length / 3) / 2
    radius = (half_length**2 + half_width**2) ** 0.5

    # ceil up to 1 digit
    # radius_disc = np.ceil(radius * 10) / 10
    radius_disc = radius
    dist_circles = length / 3 * 2

    return radius_disc, dist_circles


def compute_clcs_distance(
    clcs: CurvilinearCoordinateSystem,
    veh_rear_pos: np.ndarray,
    veh_front_pos: np.ndarray,
) -> Tuple[float, float]:
    """
    Compute the distance between two vehicles along the curvilinear coordinate system. The sign of the distance
    is based on the assumption of the relative position of the vehicles. If the distance > 0, the relative
    position relationship holds. And vice versa.

    :param clcs: curvi-linear coordinate system
    :param veh_rear_pos: the position of the rear vehicle
    :param veh_front_pos: the position of the front vehicle

    :return the longitudinal and lateral relative distances
    """
    rear_s, rear_d = clcs.convert_to_curvilinear_coords(
        veh_rear_pos[0], veh_rear_pos[1]
    )
    front_s, front_d = clcs.convert_to_curvilinear_coords(
        veh_front_pos[0], veh_front_pos[1]
    )
    return front_s - rear_s, front_d - rear_d


def compute_jerk(
    current_acceleration: float, next_acceleration: float, dt: float
) -> float:
    """
    Computes jerk given acceleration

    :param current_acceleration: acceleration of current time step
    :param next_acceleration: acceleration of previous time step
    :param dt: time step size
    :return: jerk
    """
    jerk = (next_acceleration - current_acceleration) / dt
    return jerk


def compute_acceleration(current_velocity: float, next_velocity: float, dt: float):
    """
    Computes acceleration given velocity

    :param current_velocity: velocity of current time step
    :param next_velocity: velocity of previous time step
    :param dt: timestep size
    :return: acceleration
    """
    acceleration = (next_velocity - current_velocity) / dt
    return acceleration


def compute_lanelet_width_orientation(
    lanelet: Lanelet, position: np.ndarray
) -> Tuple[Union[float, None], Union[float, None]]:
    """
    Computes the width and the orientation of the lanelet at given position

    :param lanelet: a lanelet
    """
    # smooth the vertices first:
    try:
        center_vertices = smoothing_reference_path(lanelet.center_vertices, 5, 15)
        left_vertices = smoothing_reference_path(lanelet.left_vertices, 5, 15)
        right_vertices = smoothing_reference_path(lanelet.right_vertices, 5, 15)
    except (
        TypeError,
        ValueError,
    ) as e:  # Replace with the specific exceptions you expect
        logging.error(f"Error smoothing vertices: {e}")
        center_vertices = lanelet.center_vertices
        left_vertices = lanelet.left_vertices
        right_vertices = lanelet.right_vertices

    width_list = _compute_width_from_lanalet_boundary(left_vertices, right_vertices)
    orient_list = [
        convert_to_0_2pi(orient)
        for orient in compute_orientation_from_polyline(center_vertices)
    ]
    path_length = compute_pathlength_from_polyline(center_vertices)
    lanelet_clcs = CurvilinearCoordinateSystem(center_vertices)
    position_s, _ = lanelet_clcs.convert_to_curvilinear_coords(position[0], position[1])
    return np.interp(position_s, path_length, width_list), get_orientation_point(
        position_s, path_length, orient_list
    )


def extrapolate_resample_polyline(
    polyline: np.ndarray, step: float = 2.0
) -> np.ndarray:
    """
    Extrapolates polyline for resampling.
    """
    # extend start point
    p = np.poly1d(np.polyfit(polyline[:2, 0], polyline[:2, 1], 1))

    x = 2 * polyline[0, 0] - polyline[1, 0]
    a = np.array([[x, p(x)]])
    polyline = np.concatenate((a, polyline), axis=0)

    # extrapolate final point
    p = np.poly1d(np.polyfit(polyline[-2:, 0], polyline[-2:, 1], 1))
    # extend end point

    # this extension helps the ego vehicle can drive to the end of the lane.
    x = polyline[-1, 0] + 10 * (polyline[-1, 0] - polyline[-2, 0])
    a = np.array([[x, p(x)]])
    polyline_extend = resample_polyline(
        np.concatenate((polyline[-1, np.newaxis], a), axis=0), step=step
    )
    polyline_origin = resample_polyline(polyline, step=step)

    return np.concatenate((polyline_origin, polyline_extend[1:, :]), axis=0)


def smoothing_reference_path(
    reference_path: np.ndarray, smooth_factor=None, weight_coefficient=None
):
    reference_path_extended = extrapolate_resample_polyline(reference_path)
    # generate a smooth reference path
    transposed_reference_path = reference_path_extended.T
    # how to generate index okay
    okay = np.where(
        np.abs(np.diff(transposed_reference_path[0]))
        + np.abs(np.diff(transposed_reference_path[1]))
        > 0
    )
    xp = np.r_[transposed_reference_path[0][okay], transposed_reference_path[0][-1]]
    yp = np.r_[transposed_reference_path[1][okay], transposed_reference_path[1][-1]]

    curvature = pycrccosy.Util.compute_curvature(np.array([xp, yp]).T)
    # set weights for interpolation:
    # see details: https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splprep.html
    weights = np.exp(-weight_coefficient * (abs(curvature) - np.min(abs(curvature))))
    # B spline interpolation
    tck, u = splprep([xp, yp], s=smooth_factor, w=weights)
    # double the interpolation
    u_new = np.linspace(u.min(), u.max(), len(reference_path) * 2)
    x_new, y_new = splev(u_new, tck, der=0)
    ref_path_smooth = np.array([x_new, y_new]).transpose()
    return ref_path_smooth


def get_orientation_point(position: "float", path_length, orientation):
    # TODO: fix orientation interpolation, since the orientation is in the interval [-pi, pi].
    #  Problem arises when the orientation is around pi.
    lower_idx = np.searchsorted(path_length, position, side="right") - 1
    upper_idx = lower_idx + 1
    if (orientation[lower_idx] * orientation[upper_idx] <= 0) and (
        (abs(orientation[lower_idx]) + abs(orientation[upper_idx])) > np.pi
    ):
        delta_angle = 2 * np.pi - (
            abs(orientation[lower_idx]) + abs(orientation[upper_idx])
        )
        orientation = orientation[lower_idx] + (position - path_length[lower_idx]) / (
            path_length[upper_idx] - path_length[lower_idx]
        ) * delta_angle * np.sign(path_length[lower_idx])
        if orientation < -np.pi:
            orientation = 2 * np.pi - orientation
        elif orientation > np.pi:
            orientation = orientation - 2 * np.pi
    else:
        orientation = np.interp(position, path_length, orientation)
    return orientation


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
        width_along_lanelet[i] = np.linalg.norm(left_polyline[i] - right_polyline[i])
    return width_along_lanelet


def create_polygon(
    obstacle: DynamicObstacle,
    time_step: int,
    w: float = 0,
    l_front: float = 0,
    l_back: float = 0,
) -> Polygon:
    """
    Computes the shapely-polygon of an obstacle/vehicle. Will keep minimum shape of the object, but can be extended by
    providing different values, if they are bigger than the original values.

    :param obstacle: obstacle of which the polygon should be calculated
    :param time_step: point in time in scenario
    :param w: new width
    :param l_front: extended length to the front, measured from the center
    :param l_back: extended length to the back, measured from the center
    :return: shapely-polygon of the obstacle
    """
    pos = obstacle.state_at_time(time_step).position
    angle = obstacle.state_at_time(time_step).orientation
    angle_cos = math.cos(angle)
    angle_sin = math.sin(angle)
    width = max(obstacle.obstacle_shape.width * 0.5, w)
    length_front = max(obstacle.obstacle_shape.length * 0.5, l_front)
    length_back = max(obstacle.obstacle_shape.length * 0.5, l_back)
    coords = [
        (
            pos[0] + length_front * angle_cos - width * angle_sin,
            pos[1] + length_front * angle_sin + width * angle_cos,
        ),
        (
            pos[0] - length_back * angle_cos - width * angle_sin,
            pos[1] - length_back * angle_sin + width * angle_cos,
        ),
        (
            pos[0] - length_back * angle_cos + width * angle_sin,
            pos[1] - length_back * angle_sin - width * angle_cos,
        ),
        (
            pos[0] + length_front * angle_cos + width * angle_sin,
            pos[1] + length_front * angle_sin - width * angle_cos,
        ),
        (
            pos[0] + length_front * angle_cos - width * angle_sin,
            pos[1] + length_front * angle_sin + width * angle_cos,
        ),
    ]
    return Polygon(coords)


def convert_to_0_2pi(angle, epsilon=1e-5):
    # dealing with floating-point precision errors
    if abs(angle) < epsilon:
        angle = 0.0
    # If angle is negative, convert to [0, 2π]
    elif angle < 0:
        angle += 2 * math.pi

    # If angle is more than 2π, convert to [0, 2π]
    angle = angle % (2 * math.pi)
    return angle
