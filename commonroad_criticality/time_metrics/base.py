import math
from abc import ABC, abstractmethod
from typing import List, Union
import matplotlib.pyplot as plt

# CommonRoad STL monitor
from stl_crmonitor.crmonitor.common.world_state import WorldState

# CommonRoad Toolbox
from commonroad.scenario.obstacle import DynamicObstacle, Shape
from commonroad.scenario.trajectory import State
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker, \
    create_collision_object
import commonroad_dc.boundary.boundary as boundary
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_dc.collision.visualization.drawing \
    import draw_collision_rectobb

from commonroad_criticality.time_metrics.utils import transfer_state_list_to_prediction


class CutOffBase(ABC):
    """
        Abstract base class for calculating cut-off states
    """
    def __init__(self,
                 world_state: WorldState):
        self._world_state = world_state
        self.scenario = self._world_state.scenario
        self._ego_vehicle = self.scenario.obstacle_by_id(world_state.ego_vehicle.id)
        self._N = self._world_state.num_time_steps
        self._dT = world_state.dt
        self._visualize = False
        if self.scenario.obstacle_by_id(self._ego_vehicle.obstacle_id) is not None:
            self.scenario.remove_obstacle(self._ego_vehicle)
        road_boundary_obstacle, road_boundary_sg_rectangles = boundary.create_road_boundary_obstacle(self.scenario)
        self.scenario.add_objects(road_boundary_obstacle)
        self._collision_checker = create_collision_checker(self.scenario)
        self.scenario.remove_obstacle(road_boundary_obstacle)
        if self._visualize:
            # visualize scenario and collision objects
            self.rnd = MPRenderer(figsize=(25, 10))
            self.scenario.lanelet_network.draw(self.rnd)
        # create the shape of the ego vehicle
        self._shape = self._ego_vehicle.obstacle_shape

    @property
    def world_state(self) -> WorldState:
        return self._world_state

    @property
    def ego_vehicle(self) -> DynamicObstacle:
        return self._ego_vehicle

    @property
    def shape(self) -> Shape:
        return self._shape

    @property
    def dT(self) -> float:
        return self._dT

    @dT.setter
    def dT(self, dT: float):
        raise Exception("You are not allowed to change the time step!")

    @property
    def N(self) -> int:
        return self._N

    @N.setter
    def N(self, N: int):
        raise Exception("You are not allowed to change the number of time steps!")

    @abstractmethod
    def generate(self, *args, **kwargs):
        """
        generates the cut off state: time-to-react or time-to-compliance
        """
        pass

    def _calc_ttc(self, state_list: List[State]):
        """
        Detects the collision time given the trajectory of ego_vehicle using a for loop over
        the state list.
        """
        for i in range(len(state_list)):
            # ith time step
            pos1 = state_list[i].position[0]
            pos2 = state_list[i].position[1]
            theta = state_list[i].orientation
            # i: time_start_idx
            ego = pycrcc.TimeVariantCollisionObject(i)
            ego.append_obstacle(pycrcc.RectOBB(0.5 * self._shape.length,
                                               0.5 * self._shape.width,
                                               theta, pos1, pos2))
            if self._collision_checker.collide(ego):
                if self._visualize:
                    rnd = MPRenderer()
                    ego_obb = pycrcc.RectOBB(0.5 * self._shape.length,
                                             0.5 * self._shape.width,
                                             theta, pos1, pos2)
                    draw_collision_rectobb(ego_obb, rnd)
                    self.scenario.draw(rnd, draw_params={'time_begin': i, })
                    rnd.render()
                    plt.show()
                return i*self.dT
        return math.inf

    def _detect_collision(self, state_list: Union[State]) -> bool:
        """
        return whether the state list of the ego vehicle is collision-free
        """
        # create a TrajectoryPrediction object consisting of the trajectory and the shape of the ego vehicle
        traj_pred = transfer_state_list_to_prediction(state_list, self._shape, self.dT)
        # create a collision object using the trajectory prediction of the ego vehicle
        co = create_collision_object(traj_pred)
        return self._collision_checker.collide(co)

