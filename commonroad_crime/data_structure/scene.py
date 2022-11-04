
__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from commonroad.scenario.scenario import Scenario, LaneletNetwork
from typing import Union, List
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle


class Scene(Scenario):
    """
    a snapshot of the environment including the scenery and dynamic elements, as well as all actors’ and observers’
    self-representations, and the relationships among those entities
    """

    def __init__(self, time_step: int, scenario: Scenario, dt: float=None,
                 scene_id: Union[str, None] = None):
        if dt is None:
            dt = scenario.dt
        super().__init__(dt)
        if scene_id is None:
            self.scene_id = str(scenario.scenario_id) + '_' + str(time_step)
        else:
            self.scene_id = scene_id
        self._lanelet_network = scenario.lanelet_network
        self.time_step = time_step
        self._scenario = scenario

    @property
    def static_obstacles(self) -> List[StaticObstacle]:
        """ Returns a list of all static obstacles in the scene."""
        return [stat_obs for stat_obs in self._scenario.static_obstacles if
                stat_obs.state_at_time(self.time_step) is not None]

    @property
    def dynamic_obstacles(self) -> List[DynamicObstacle]:
        """ Returns a list of all static obstacles in the scene."""
        return [dyn_obs for dyn_obs in self._scenario.dynamic_obstacles if
                dyn_obs.state_at_time(self.time_step) is not None]

    @property
    def obstacles(self) -> List[Union[StaticObstacle, DynamicObstacle]]:
        """ Returns a list of all obstacles roles in the scene."""
        return [*self.static_obstacles, *self.dynamic_obstacles]
