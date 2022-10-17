__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.scenario import Scenario
from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Union
from pathlib import Path


def load_scenario(config) -> Scenario:
    """
    Loads a scenario from the configuration.

    :param config: configuration
    :return: scenario
    """
    scenario, _ = CommonRoadFileReader(config.general.path_scenario).open()
    return scenario


def save_fig(metric_name: str, path_output: str, time_step: int):
    # save as svg
    Path(path_output).mkdir(parents=True, exist_ok=True)
    plt.savefig(f'{path_output}{metric_name}_{time_step:03d}.svg', format="svg", bbox_inches="tight",
                transparent=False)


def compute_lanelet_width(lanelet: Lanelet, position: List[float]) -> Union[float, None]:
    """
    Computes the width of the lanelet at given position

    :param lanelet: a lanelet
    :param position: position of the vehicle
    """
    if not lanelet.contains_points(np.array([position])):
        return None
    width_list = _compute_width_from_lanalet_boundary(lanelet.left_vertices, lanelet.right_vertices)
    path_length = _compute_path_length_from_polyline(lanelet.center_vertices)
    lanelet_clcs = CurvilinearCoordinateSystem(lanelet.center_vertices)
    position_s, _ = lanelet_clcs.convert_to_curvilinear_coords(position[0], position[1])
    return np.interp(position_s, path_length, width_list)


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
