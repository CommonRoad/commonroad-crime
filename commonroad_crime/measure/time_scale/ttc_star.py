__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import copy
import math
import logging
from typing import List
import matplotlib.pyplot as plt

from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.scenario import State, TrajectoryPrediction, Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.geometry.shape import ShapeGroup

import commonroad_dc.boundary.boundary as boundary
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import (create_collision_checker,
                                                                                   create_collision_object)

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTimeScale
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class TTCStar(CriMeBase):
    measure_name = TypeTimeScale.TTCStar

    def __init__(self, config: CriMeConfiguration):
        super(TTCStar, self).__init__(config)
        self.sce.remove_obstacle(self.ego_vehicle)

        # creat collision checker
        road_boundary_obstacle, _ = boundary.create_road_boundary_obstacle(self.sce,
                                                                           method='aligned_triangulation',
                                                                           axis=2)
        self.sce.add_objects(road_boundary_obstacle)
        self.collision_checker = create_collision_checker(self.sce)
        # remove the added objects from the scenario
        self.sce.remove_obstacle(road_boundary_obstacle)
        self.sce.add_objects(self.ego_vehicle)

    def detect_collision(self, state_list: List[State]) -> bool:
        """
        Returns whether the state list of the ego vehicle is collision-free.

        :param state_list: list of vehicle states
        """
        # update the trajectory prediction
        updated_ego_vehicle = copy.deepcopy(self.ego_vehicle)
        dynamic_obstacle_trajectory = Trajectory(state_list[0].time_step, state_list)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory,
                                                           updated_ego_vehicle.obstacle_shape)
        updated_ego_vehicle.prediction = dynamic_obstacle_prediction
        co = create_collision_object(updated_ego_vehicle)

        return self.collision_checker.collide(co)

    def draw_collision_checker(self, rnd: MPRenderer):
        """
        Plots the collision checker.
        """
        self.collision_checker.draw(rnd,
                                    draw_params={'facecolor': TUMcolor.TUMgray,
                                                 'edgecolor': TUMcolor.TUMdarkgray ,'draw_mesh': False})

    def visualize(self, figsize: tuple = (25, 15)):
        self._initialize_vis(figsize=figsize,
                             plot_limit=[30, 80, -3.5, 7],)# [65, 105, -4, 7.5],
        # self.draw_collision_checker(self.rnd)
        if self.value not in [math.inf, -math.inf]:
            tstc = int(utils_gen.int_round(self.value / self.dt, 0))
            utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, tstc)
            utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(tstc), TUMcolor.TUMred)
        else:
            tstc = self.value
        plt.title(f"time step: {tstc}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, tstc)
        else:
            plt.show()

    def compute(self, time_step: int = 0, vehicle_id: int = None):
        """
        Detects the collision time given the trajectory of ego_vehicle using a for loop over
        the state list.
        """
        self.time_step = time_step
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        state_list = self.ego_vehicle.prediction.trajectory.state_list
        self.value = math.inf
        for i in range(time_step, len(state_list)):
            # i-th time step
            pos1 = self.ego_vehicle.state_at_time(i).position[0]
            pos2 = self.ego_vehicle.state_at_time(i).position[1]
            theta = self.ego_vehicle.state_at_time(i).orientation
            # i: time_start_idx
            ego = pycrcc.TimeVariantCollisionObject(i)
            ego.append_obstacle(pycrcc.RectOBB(0.5 * self.ego_vehicle.obstacle_shape.length,
                                               0.5 * self.ego_vehicle.obstacle_shape.width,
                                               theta, pos1, pos2))
            flag_collide = self.collision_checker.collide(ego)
            if flag_collide:
                self.value = utils_gen.int_round((i - time_step) * self.dt, str(self.dt)[::-1].find('.'))
                # once collides, loop ends -> the first colliding timestep as the ttc
                break
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value