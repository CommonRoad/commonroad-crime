__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import copy
import numpy as np
import logging
from typing import Dict, List, Union

from commonroad.scenario.state import InitialState, CustomState

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.type import TypeReachableSet
from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log

from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.pycrreach import ReachNode
from commonroad_reach.utility import visualization as util_visual
import commonroad_reach.utility.reach_operation as utils_ops

logger = logging.getLogger(__name__)


class DA(CriMeBase):
    """
    Klischat, Moritz, et al. "Scenario factory: Creating safety-critical traffic scenarios for automated vehicles."
    2020 IEEE 23rd International Conference on Intelligent Transportation Systems (ITSC). IEEE, 2020.
    """

    measure_name = TypeReachableSet.DA

    def __init__(self, config: CriMeConfiguration):
        super(DA, self).__init__(config)
        self.reach_config = ConfigurationBuilder().build_configuration(
            config.general.name_scenario
        )
        # update the paths based on CriMe
        self.reach_config.general.path_scenario = (
            self.configuration.general.path_scenario
        )
        # update the paths
        self.reach_config.general.path_output = self.configuration.general.path_output
        self.reach_config.general.path_scenarios = (
            self.configuration.general.path_scenarios
        )
        self.reach_config.planning.steps_computation = (
            self.configuration.reachable_set.time_horizon
        )
        self.reach_config.planning.dt = self.sce.dt
        if self.configuration.reachable_set.coordinate_system == 1:
            self.reach_config.planning.coordinate_system = "CART"
        self.reach_config.update(scenario=self.sce)
        self.reach_interface = ReachableSetInterface(self.reach_config)

    def _update_initial_state(self, target_state: Union[InitialState, CustomState]):
        self.reach_config.planning_problem.initial_state.position = (
            target_state.position
        )
        if hasattr(target_state, "velocity_y"):
            self.reach_config.planning_problem.initial_state.velocity = np.sqrt(
                target_state.velocity**2 + target_state.velocity_y**2
            )
        else:
            self.reach_config.planning_problem.initial_state.velocity = (
                target_state.velocity
            )
        self.reach_config.planning_problem.initial_state.orientation = (
            target_state.orientation
        )
        self.reach_config.planning_problem.initial_state.time_step = (
            target_state.time_step
        )

    def compute(self, time_step: int = 0, vehicle_id: int = None, verbose: bool = True):
        self.value = 0.0
        evaluated_state = copy.deepcopy(self.ego_vehicle.state_at_time(time_step))
        self._update_initial_state(target_state=evaluated_state)
        self.reach_config.update(planning_problem=self.reach_config.planning_problem)
        self.reach_config.scenario.remove_obstacle(
            self.reach_config.scenario.obstacle_by_id(self.ego_vehicle.obstacle_id)
        )
        self.reach_interface.reset(self.reach_config)
        self.reach_interface.compute_reachable_sets(verbose=verbose)
        self.value = compute_drivable_area(self.reach_interface.reachable_set)
        self.value = utils_gen.int_round(self.value, 2)
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}", verbose
        )
        return self.value

    def visualize(self):
        util_visual.plot_scenario_with_reachable_sets(self.reach_interface)
        # util_visual.plot_collision_checker(self.reach_interface)


def compute_drivable_area_profile(
    reachable_set: Dict[int, List[ReachNode]]
) -> np.ndarray:
    """
    Computes area profile for given reachability analysis.
    """
    area_profile = []
    for t, reach_set_nodes in reachable_set.items():
        area_profile.append(utils_ops.compute_area_of_reach_nodes(reach_set_nodes))
    return np.array(area_profile)


def compute_drivable_area(reachable_set: Dict[int, List[ReachNode]]):
    """
    Computes drivable area.
    """
    area_profile = compute_drivable_area_profile(reachable_set)
    return np.sum(area_profile)
