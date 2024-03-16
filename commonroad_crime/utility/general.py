__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.2"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import (
    State,
    KSState,
    LongitudinalState,
    PMInputState,
    PMState,
    CustomState,
)
from commonroad.scenario.scenario import Scenario, DynamicObstacle, StaticObstacle
from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_dc.geometry.util import chaikins_corner_cutting, resample_polyline
import commonroad_dc.pycrccosy as pycrccosy

import commonroad_crime.utility.solver as utils_sol

import numpy as np
import math
import logging
from typing import List, Union
import functools
from scipy.interpolate import splprep, splev

logger = logging.getLogger(__name__)


def load_scenario(config) -> Scenario:
    """
    Loads a scenario from the configuration.

    :param config: configuration
    :return: scenario
    """
    scenario, _ = CommonRoadFileReader(config.general.path_scenario).open(
        lanelet_assignment=True
    )
    return scenario


def generate_reference_path(
    lanelet_id: int, lanelet_network: LaneletNetwork, flag_resampling=True
):
    """
    Generate the reference path based on the center line of the provided lanelet.
    """
    ini_lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
    ref_path = ini_lanelet.center_vertices
    # extend the reference path
    pre_lanelet = ini_lanelet
    i = 0
    while pre_lanelet.predecessor:
        pre_lanelet = lanelet_network.find_lanelet_by_id(pre_lanelet.predecessor[0])
        ref_path = np.concatenate((pre_lanelet.center_vertices, ref_path))
        i += 1
        if i >= 2:
            break
    suc_lanelet = ini_lanelet
    i = 0
    while suc_lanelet.successor:
        suc_lanelet = lanelet_network.find_lanelet_by_id(suc_lanelet.successor[0])
        ref_path = np.concatenate((ref_path, suc_lanelet.center_vertices))
        i += 1
        if i >= 2:
            break
    if flag_resampling:
        ref_path = np.array(chaikins_corner_cutting(ref_path))
        ref_path = resample_polyline(ref_path)
    return ref_path


@functools.lru_cache()
def int_round(some_float, tolerance=1):
    """
    Round function using int.

    :param some_float: number
    :param tolerance: float point
    :return: rounded number
    """
    p = float(10**tolerance)
    if some_float < 0:
        return int(some_float * p - 0.5) / p
    else:
        return int(some_float * p + 0.5) / p


def check_in_same_lanelet(
    lanelet_network: LaneletNetwork,
    vehicle_1: DynamicObstacle,
    vehicle_2: Union[DynamicObstacle, StaticObstacle],
    time_step: int,
):
    if not vehicle_1.occupancy_at_time(time_step):
        logger.info(
            f"<utility> vehicle {vehicle_1.obstacle_id} doesn't have occupancies at time step {time_step}."
        )
        return False

    if not vehicle_2.occupancy_at_time(time_step):
        logger.info(
            f"<utility> vehicle {vehicle_2.obstacle_id} doesn't have occupancies at time step {time_step}."
        )
        return False

    # Proceed only if both vehicles have occupancies
    lanelets_1 = lanelet_network.find_lanelet_by_shape(
        vehicle_1.occupancy_at_time(time_step).shape
    )
    lanelets_2 = lanelet_network.find_lanelet_by_shape(
        vehicle_2.occupancy_at_time(time_step).shape
    )

    # Check if there is an intersection between the sets of lanelets occupied by the vehicles
    return len(set(lanelets_1).intersection(lanelets_2)) > 0


def check_elements_state_list(
    state_list: List[
        Union[LongitudinalState, KSState, CustomState, PMState, State, None]
    ],
    dt: float,
):
    v_list = [state.velocity for state in state_list]
    t_list = [state.time_step * dt for state in state_list]
    a_list = np.gradient(np.array(v_list), t_list)
    j_list = np.gradient(np.array(a_list), t_list)
    for i in range(len(state_list)):
        state_list[i].acceleration = a_list[i]
        state_list[i].jerk = j_list[i]
        check_elements_state(state_list[i], dt=dt)


def check_elements_state(
    state: Union[KSState, LongitudinalState, PMState, CustomState, State],
    veh_input: PMInputState = None,
    next_state: Union[KSState, LongitudinalState, State] = None,
    dt: float = 0.1,
):
    """
    checks the missing elements needed for PM model
    """
    if not hasattr(state, "slip_angle"):
        state.slip_angle = 0
    if not hasattr(state, "yaw_rate"):
        state.yaw_rate = 0
    if not hasattr(state, "orientation"):
        state.orientation = math.atan2(state.velocity_y, state.velocity)
    if not hasattr(state, "velocity_y") and hasattr(state, "velocity"):
        state.velocity_y = state.velocity * math.sin(state.orientation)
        state.velocity = state.velocity * math.cos(state.orientation)

    # check the acceleration
    if not hasattr(state, "acceleration"):
        if next_state:
            state.acceleration = utils_sol.compute_acceleration(
                state.velocity, next_state.velocity, dt
            )
        else:
            state.acceleration = 0.0
        state.jerk = 0.0
    else:
        if next_state:
            if hasattr(next_state, "acceleration"):
                state.jerk = utils_sol.compute_jerk(
                    state.acceleration, next_state.acceleration, dt
                )
    if veh_input is not None:
        state.acceleration = veh_input.acceleration
        state.acceleration_y = veh_input.acceleration_y
    if not hasattr(state, "orientation"):
        ref_orientation = math.atan2(state.velocity_y, state.velocity)
    else:
        ref_orientation = state.orientation
    if not hasattr(state, "acceleration_y") and hasattr(state, "acceleration"):
        if state.acceleration is not None:
            state.acceleration_y = state.acceleration * math.sin(ref_orientation)
            state.acceleration = state.acceleration * math.cos(ref_orientation)


def compute_curvature_from_polyline_start_end(polyline: np.ndarray) -> float:
    """
    idea: assume the turning lane is a part of circle, calculate the curvature based on the function of chord
    """
    assert (
        isinstance(polyline, np.ndarray)
        and polyline.ndim == 2
        and len(polyline[:, 0]) > 2
    ), "Polyline malformed for curvature computation p={}".format(polyline)

    # fixme: the weight factors might need to be adjusted
    _, kappa = smoothing_reference_path(polyline, 0.1, 1)
    return kappa


def smoothing_reference_path(
    reference_path: np.ndarray, smooth_factor=None, weight_coefficient=None
):
    """Smooth the reference path"""
    # generate a smooth reference path
    transposed_reference_path = reference_path.T
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
    u_new = np.linspace(u.min(), u.max(), 2000)
    x_new, y_new = splev(u_new, tck, der=0)
    ref_path_smooth = np.array([x_new, y_new]).transpose()
    orientation_start = np.arctan2(y_new[1] - y_new[0], x_new[1] - x_new[0])
    orientation_end = np.arctan2(y_new[-1] - y_new[-2], x_new[-1] - x_new[-2])
    delta_orientation = utils_sol.convert_to_0_2pi(orientation_end - orientation_start)
    distance = np.sqrt((x_new[-1] - x_new[0]) ** 2 + (y_new[-1] - y_new[0]) ** 2)
    r = distance / (2 * np.sin(delta_orientation / 2))
    kappa = 1 / r
    return ref_path_smooth, kappa
