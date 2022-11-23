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
from commonroad_dc import pycrcc
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTimeScale
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_dc.boundary.boundary as boundary

logger = logging.getLogger(__name__)


class TTC(CriMeBase):
    """
    With a constant acceleration decision model of the vehicles motion

    -- using (5.24) in "Collision Avoidance Theory with Application to Automotive Collision Mitigation"
    """
    metric_name = TypeTimeScale.TTC

    def __init__(self, config: CriMeConfiguration):
        super(TTC, self).__init__(config)
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

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        self._set_other_vehicles(vehicle_id)
        self.time_step = time_step
        state_list = self.ego_vehicle.prediction.trajectory.state_list
        pos1 = state_list[0].position[0]
        pos2 = state_list[0].position[1]
        vx = state_list[0].velocity
        vy = state_list[0].velocity_y
        theta = state_list[0].orientation
        ######### to be implemented ##############
        self.value = 0.5 #math.inf
        for i in range(time_step, len(state_list)):
            """
            Evaluation of collisions should be the same as in ttcstar, only the calculation of the state is different.
            Instead of getting the state from the state_list, we have a fixed orientation and fixed velocity.
            So for each time step we derive the new position by the velocity * dt added to the old position
            """
            ego = pycrcc.TimeVariantCollisionObject(i)
            ego.append_obstacle(pycrcc.RectOBB(0.5 * self.ego_vehicle.obstacle_shape.length,
                                               0.5 * self.ego_vehicle.obstacle_shape.width,
                                               theta, pos1, pos2))
            ego_obb = pycrcc.RectOBB(0.5 * self.ego_vehicle.obstacle_shape.length,
                                     0.5 * self.ego_vehicle.obstacle_shape.width,
                                     theta, pos1, pos2)
            ego.append_obstacle(ego_obb)
            if self.collision_checker.collide(ego):
                self.value = utils_gen.int_round((i - time_step) * self.dt, str(self.dt)[::-1].find('.'))
                # once collides, loop ends -> the first colliding timestep as the ttc
                break

            pos1 += self.dt * vx
            pos2 += self.dt * vy
        ##########################################
        utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
        return self.value

    def visualize(self):
        self._initialize_vis(plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                              self.ego_vehicle.prediction.
                                                                              trajectory.state_list,
                                                                              margin=10))
        self.other_vehicle.draw(self.rnd, {'time_begin': self.time_step, **utils_vis.OTHER_VEHICLE_DRAW_PARAMS})
        self.rnd.render()
        plt.title(f"{self.metric_name} at time step {self.time_step}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()

