import copy
import logging

from commonroad.scenario.scenario import State

try:
    from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
    from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
    from commonroad_reach.pycrreach import ReachPolygon, ReachNode
    from commonroad_reach.utility import visualization as util_visual
except ModuleNotFoundError:
    raise ModuleNotFoundError('commonroad_reach is not installed')

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.metric import ReachableSetScaleMetricType
from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.solver as utils_sol

logger = logging.getLogger(__name__)


class DrivableArea(CriMeBase):
    """
    Klischat, Moritz, et al. "Scenario factory: Creating safety-critical traffic scenarios for automated vehicles."
    2020 IEEE 23rd International Conference on Intelligent Transportation Systems (ITSC). IEEE, 2020.
    """
    metric_name = ReachableSetScaleMetricType.DA

    def __init__(self, config: CriMeConfiguration):
        super(DrivableArea, self).__init__(config)
        self.reach_config = ConfigurationBuilder.build_configuration(config.general.name_scenario)
        # update the paths based on CriMe
        self.reach_config.general.path_scenario = self.configuration.general.path_scenario
        self.reach_config.general.path_output = self.configuration.general.path_output
        self.reach_config.planning.steps_computation = self.configuration.reachable_set_scale.time_horizon
        self.reach_config.update()
        self.reach_interface = ReachableSetInterface(self.reach_config)

    def _update_initial_state(self, target_state: State):
        self.reach_config.planning_problem.initial_state.position = target_state.position
        self.reach_config.planning_problem.initial_state.velocity = target_state.velocity
        self.reach_config.planning_problem.initial_state.orientation = target_state.orientation

    def compute(self, time_step: int = 0, verbose: bool = True):
        self.value = 0.
        evaluated_state = copy.deepcopy(self.ego_vehicle.state_at_time(time_step))
        self._update_initial_state(target_state=evaluated_state)
        self.reach_config.update(planning_problem=self.reach_config.planning_problem)
        self.reach_config.scenario.remove_obstacle(
            self.reach_config.scenario.obstacle_by_id(self.ego_vehicle.obstacle_id))
        self.reach_interface.reset(self.reach_config)
        self.reach_interface.compute_reachable_sets()
        self.value = utils_sol.compute_drivable_area(self.reach_interface.reachable_set)
        self.value = utils_gen.int_round(self.value, 2)

        return self.value

    def visualize(self):
        util_visual.plot_scenario_with_reachable_sets(self.reach_interface)
