__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import copy
import math
from typing import Union, List
from decimal import Decimal
import matplotlib.pyplot as plt

from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.scenario import State, TrajectoryPrediction
from commonroad.scenario.trajectory import Trajectory

import commonroad_dc.boundary.boundary as boundary
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import (create_collision_checker,
create_collision_object)
from commonroad_dc.collision.visualization.drawing \
    import draw_collision_rectobb

from commonroad_criticality.data_structure.base import CriticalityBase
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
import commonroad_criticality.utility.visualization as Utils_vis
import commonroad_criticality.utility.general as Utils_gen


class TTC(CriticalityBase):
    metric_name = TimeScaleMetricType.TTC

    def __init__(self, config: CriticalityConfiguration):
        super(TTC, self).__init__(config)
        self.sce.remove_obstacle(self.ego_vehicle)

        # creat collision checker
        road_boundary_obstacle, road_boundary_sg_rectangles = boundary.create_road_boundary_obstacle(self.sce)
        self.sce.add_objects(road_boundary_obstacle)
        self.collision_checker = create_collision_checker(self.sce)
        self._colliding_ego_obb = None

    def detect_collision(self, state_list: List[State]) -> bool:
        """
        Returns whether the state list of the ego vehicle is collision-free.
        """
        # update the trajectory prediction
        updated_ego_vehicle = copy.deepcopy(self.ego_vehicle)
        dynamic_obstacle_trajectory = Trajectory(state_list[0].time_step, state_list)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory,
                                                           updated_ego_vehicle.obstacle_shape)
        updated_ego_vehicle.prediction = dynamic_obstacle_prediction
        co = create_collision_object(updated_ego_vehicle)
        return self.collision_checker.collide(co)

    def visualize(self):
        if self.configuration.debug.draw_visualization:
            rnd = MPRenderer()
            if self._colliding_ego_obb:
                draw_collision_rectobb(self._colliding_ego_obb, rnd)
            if self.value not in [math.inf, -math.inf]:
                tstc = int(Utils_gen.int_round(self.value / self.dt, 0))
                self.sce.draw(rnd, draw_params={'time_begin': tstc,
                                                "dynamic_obstacle": {
                                                    "draw_icon": self.configuration.debug.draw_icons}})
            rnd.render()
            plt.title(f"time step: {self.value}")
            if self.configuration.debug.save_plots:
                Utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.value)
            else:
                plt.show()

    def compute(self) -> Union[int, float]:
        """
        Detects the collision time given the trajectory of ego_vehicle using a for loop over
        the state list.
        """
        state_list = self.ego_vehicle.prediction.trajectory.state_list
        self.value = math.inf
        for i in range(len(state_list)):
            # ith time step
            pos1 = state_list[i].position[0]
            pos2 = state_list[i].position[1]
            theta = state_list[i].orientation
            # i: time_start_idx
            ego = pycrcc.TimeVariantCollisionObject(i)
            ego.append_obstacle(pycrcc.RectOBB(0.5 * self.ego_vehicle.obstacle_shape.length,
                                               0.5 * self.ego_vehicle.obstacle_shape.width,
                                               theta, pos1, pos2))
            ego_obb = pycrcc.RectOBB(0.5 * self.ego_vehicle.obstacle_shape.length,
                                     0.5 * self.ego_vehicle.obstacle_shape.width,
                                     theta, pos1, pos2)
            ego.append_obstacle(ego_obb)
            if self.collision_checker.collide(ego):
                self._colliding_ego_obb = ego_obb
                self.value = Utils_gen.int_round(i * self.dt, 1)
                # once collides, loop ends -> the first colliding timestep as the ttc
                break
        return self.value
