from typing import Tuple, List, Dict
import numpy as np

from commonroad.scenario.obstacle import Obstacle, StaticObstacle


try:
    from commonroad_reach.pycrreach import ReachPolygon, ReachNode
    import commonroad_reach.utility.reach_operation as utils_ops
except ModuleNotFoundError:
    raise ModuleNotFoundError('commonroad_reach is not installed')

def solver_wttc(veh_1: Obstacle,
                veh_2: Obstacle,
                time_step: int,
                a_max: float,):
    """
    Analytical solution of the worst-time-to-collision.
    """
    r_v1, _ = compute_disc_radius_and_distance(veh_1.obstacle_shape.length,
                                               veh_1.obstacle_shape.width)
    r_v2, _ = compute_disc_radius_and_distance(veh_2.obstacle_shape.length,
                                               veh_2.obstacle_shape.width)
    x_10, y_10 = veh_1.state_at_time(time_step).position
    x_20, y_20 = veh_2.state_at_time(time_step).position
    v_1x0, v_1y0 = veh_1.state_at_time(time_step).velocity * np.array(
        [np.cos(veh_1.state_at_time(time_step).orientation),
         np.sin(veh_1.state_at_time(time_step).orientation)])
    v_2x0, v_2y0 = veh_2.state_at_time(time_step).velocity * np.array(
        [np.cos(veh_2.state_at_time(time_step).orientation),
         np.sin(veh_2.state_at_time(time_step).orientation)])
    a_10 = a_20 = a_max
    if isinstance(veh_2, StaticObstacle):
        a_20 = 0
    # compute the parameters
    A = -1/4 * (a_10 + a_20) ** 2
    B = 0
    C = -(a_20 + a_10) * (r_v1 + r_v2) + (v_2x0 - v_1x0) ** 2 + (v_2y0 - v_1y0) ** 2
    D = 2 * (v_2x0 - v_1x0) * (x_20 - x_10) + 2 * (v_2y0 - v_1y0) * (y_20 - y_10)
    E = (x_20 - x_10) ** 2 + (y_20 - y_10) ** 2 - (r_v1 + r_v2) ** 2
    return np.roots(np.array([A, B, C, D, E]))


def compute_disc_radius_and_distance(length: float, width: float, ref_point="CENTER", dist_axle_rear=None) \
        -> Tuple[float, float]:
    """
    Computes the radius of discs and their distances used as the approximation of the shape of the ego vehicle.
    Credits: Gerald Würsching.

    .. note::
        Vehicle occupancy is approximated by three equally sized discs with equidistant center points.
        (see Ziegler, J. and Stiller, C. (2010) **"Fast collision checking for intelligent vehicle motion planning"**,
        IEEE IV

    :param length: vehicle length
    :param width: vehicle width
    :param ref_point: "CENTER" or "REAR"
    :param dist_axle_rear: if ref_point == "REAR", the distance between vehicle center and rear axle has to be provided
    :return: radius_disc: radius of discs
    :return: dist_circles: distance between the first and the third circle
    """
    assert length >= 0 and width >= 0, f"Invalid vehicle dimensions: length = {length}, width = {width}"

    if np.isclose(length, 0.0) and np.isclose(width, 0.0):
        return 0.0, 0.0

    # half width of the ego vehicle
    half_width = width / 2

    if ref_point == "CENTER":
        # second circle center point is exactly at the geometric center of vehicle model
        # the other circles are placed equidistant along the longitudinal axis
        half_length = (length / 3) / 2
        radius = (half_length ** 2 + half_width ** 2) ** 0.5

        # ceil up to 1 digit
        # radius_disc = np.ceil(radius * 10) / 10
        radius_disc = radius
        dist_circles = length / 3 * 2

    elif ref_point == "REAR":
        # first circle center point has to be exactly on rear axis position of vehicle model
        assert dist_axle_rear >= 0, f"Please provide a valid value for the rear axle distance (dist_axle_rear = " \
                                    f"{dist_axle_rear})"
        if dist_axle_rear < length / 3:
            half_length = length / 2 - dist_axle_rear
            radius = (half_length ** 2 + half_width ** 2) ** 0.5

            # ceil up to 1 digit
            # radius_disc = np.ceil(radius * 10) / 10
            radius_disc = radius
            dist_circles = dist_axle_rear * 2

        else:
            half_length = (length / 3) / 2 + (dist_axle_rear - length / 3)
            radius = (half_length ** 2 + half_width ** 2) ** 0.5

            radius_disc = radius
            dist_circles = dist_axle_rear * 2
    else:
        raise Exception("reference point has to be either 'CENTER' or 'REAR'")

    return radius_disc, dist_circles


def compute_drivable_area_profile(reachable_set: Dict[int, List[ReachNode]]) -> np.ndarray:
    """
    Computes area profile for given reachability analysis.
    """
    area_profile = []
    for t, reach_set_nodes in reachable_set.items():
        area_profile.append(utils_ops.compute_area_of_reach_nodes(reach_set_nodes))
    return np.array(area_profile)


def compute_drivable_area(reachable_set: Dict[int, List[ReachNode]]):
    """
    Computes drivable area.
    """
    area_profile = compute_drivable_area_profile(reachable_set)
    return np.sum(area_profile)
