__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.geometry.util import chaikins_corner_cutting, resample_polyline

import numpy as np
from typing import Union, Tuple
import functools


def load_scenario(config) -> Scenario:
    """
    Loads a scenario from the configuration.

    :param config: configuration
    :return: scenario
    """
    scenario, _ = CommonRoadFileReader(config.general.path_scenario).open(lanelet_assignment=True)
    return scenario


def generate_reference_path(lanelet_id: int, lanelet_network: LaneletNetwork, flag_resampling=True):
    """
    Generate the reference path based on the center line of the provided lanelet.
    """
    ini_lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
    ref_path = ini_lanelet.center_vertices
    # extend the reference path
    pre_lanelet = ini_lanelet  # todo: more predecessors?
    while pre_lanelet.predecessor:
        pre_lanelet = lanelet_network.find_lanelet_by_id(pre_lanelet)
        ref_path = np.concatenate((pre_lanelet.center_vertices, ref_path))
    suc_lanelet = ini_lanelet  # todo: more successors?
    while suc_lanelet.successor:
        suc_lanelet = lanelet_network.find_lanelet_by_id(suc_lanelet)
        ref_path = np.concatenate((ref_path, suc_lanelet.center_vertices))
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
    p = float(10 ** tolerance)
    if some_float < 0:
        return int(some_float * p - 0.5) / p
    else:
        return int(some_float * p + 0.5) / p


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
