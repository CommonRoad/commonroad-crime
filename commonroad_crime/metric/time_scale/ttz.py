__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging
import math

import matplotlib.pyplot as plt
import numpy as np
from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle, ObstacleType, State
from commonroad.scenario.lanelet import LaneletType
from commonroad_dc import pycrcc
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTimeScale
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.visualization as utils_vis

from commonroad_crime.metric.time_scale.ttc import TTC

logger = logging.getLogger(__name__)


class TTZ(CriMeBase):
    """
    The distance to the zebra/crosswalk divided by the speed at any given moment in time.
    """
    metric_name = TypeTimeScale.TTZ

    def __init__(self, config: CriMeConfiguration):
        super(TTZ, self).__init__(config)
        self._ttc_object = TTC(config)

    def compute(self, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        self.time_step = time_step
        zebra_list = []
        for ll in self.sce.lanelet_network.lanelets:
            if ll.lanelet_type == LaneletType.CROSSWALK:
                zebra_list.append(ll)
        if zebra_list:
            ttz_list = []
            for zebra in zebra_list:
                init_state = State(**{"position": [zebra.center_vertices[int(len(zebra.center_vertices) / 2)]],
                                      "orientation": np.tan(zebra.center_vertices[1][1] - zebra.center_vertices[0][1] /
                                                            zebra.center_vertices[0][1] - zebra.center_vertices[0][0]),
                                      "velocity": 0.})
                zebra_obs = \
                    StaticObstacle(self.sce.generate_object_id(), ObstacleType.ROAD_BOUNDARY, zebra.polygon, init_state)
                self.sce.add_objects(zebra_obs)
                ttz_list.append(
                    self._ttc_object.compute(zebra_obs.obstacle_id, self.time_step)
                )
        else:
            utils_log.print_and_log_info(logger, f"*\t\t there exists no zebra")
            self.value = math.inf
        utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")

    def visualize(self):
        pass
