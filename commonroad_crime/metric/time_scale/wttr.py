__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import copy
import logging

from commonroad.scenario.scenario import State

try:
    from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
    from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
    from commonroad_reach.utility import visualization as util_visual
except ModuleNotFoundError:
    raise ModuleNotFoundError('commonroad_reach is not installed')

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.metric.time_scale.ttc import TTC
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.metric import TimeScaleMetricType
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class WTTR(CriMeBase):
    metric_name = TimeScaleMetricType.WTTR

    def __init__(self,
                 config: CriMeConfiguration):
        super(WTTR, self).__init__(config)
        self.ttc_object = TTC(config)
        self.ttc = None
        self.reach_config = ConfigurationBuilder.build_configuration(config.general.name_scenario)
        # update the paths based on CriMe
        self.reach_config.general.path_scenario = self.configuration.general.path_scenario
        self.reach_config.general.path_output = self.configuration.general.path_output
        self.reach_config.update()
        self.reach_interface = ReachableSetInterface(self.reach_config)
        self._end_sim = None

    def _update_initial_state(self, target_state: State):
        self.reach_config.planning_problem.initial_state.position = target_state.position
        self.reach_config.planning_problem.initial_state.velocity = target_state.velocity
        self.reach_config.planning_problem.initial_state.orientation = target_state.orientation

    def compute(self, time_step: int = 0, verbose: bool = False):
        self.ttc = self.ttc_object.compute(time_step)
        if self.ttc == 0:
            self.value = -math.inf
        elif self.ttc == math.inf:
            self.value = math.inf
        else:
            self.value = self.binary_search(time_step)
        if self.value in [math.inf, -math.inf]:
            utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
            return self.value
        self.value = utils_gen.int_round(self.value, str(self.dt)[::-1].find('.'))
        utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
        return self.value

    def binary_search(self, initial_step: int):
        """
        Binary search to find the last time to execute the maneuver.
        """
        wttr = - math.inf
        low = initial_step
        tstc = int(utils_gen.int_round(self.ttc / self.dt,  str(self.dt)[::-1].find('.')))
        high = tstc + initial_step
        time_end = self.ego_vehicle.prediction.final_time_step
        while low < high:
            mid = int((low + high) / 2)
            mid_state = copy.deepcopy(self.ego_vehicle.state_at_time(mid))
            self._update_initial_state(mid_state)
            self.reach_config.update(planning_problem=self.reach_config.planning_problem)
            self.reach_config.scenario.remove_obstacle(
                self.reach_config.scenario.obstacle_by_id(self.ego_vehicle.obstacle_id))
            self.reach_interface.reset(self.reach_config)
            self._end_sim = time_end - mid
            self.reach_interface.compute_reachable_sets(0, self._end_sim, verbose=True)
            if self.reach_interface.reachable_set_at_step(self._end_sim):
                # the final step is still reachable without causing the collision
                low = mid + 1
            else:
                high = mid
        if low != initial_step:
            wttr = (low - initial_step - 1) * self.dt
        return wttr

    def visualize(self):
        util_visual.plot_scenario_with_reachable_sets(self.reach_interface, step_end= self._end_sim)

