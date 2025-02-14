__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.4.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import math
import copy
import logging
import numpy as np

from commonroad.scenario.state import State

from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.measure.time.ttc_star import TTCStar
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class WTTR(CriMeBase):
    """
    Worst-case of time-to-react. Originally obtained from Sontges, Sebastian, Markus Koschi, and Matthias Althoff.
    "Worst-case analysis of the time-to-react using reachable sets." 2018 IEEE Intelligent Vehicles Symposium (IV).
    IEEE, 2018.
    """

    measure_name = TypeTime.WTTR

    def __init__(self, config: CriMeConfiguration):
        super(WTTR, self).__init__(config)
        self.ttc_object = TTCStar(config)
        self.ttc = None
        self.reach_config = ConfigurationBuilder().build_configuration(
            config.general.name_scenario
        )
        # update the paths based on CriMe
        self.reach_config.general.path_scenario = (
            self.configuration.general.path_scenario
        )
        self.reach_config.general.path_output = self.configuration.general.path_output
        self.reach_config.vehicle.ego.a_lon_min = (
            self.configuration.vehicle.curvilinear.a_lon_min
        )
        self.reach_config.vehicle.ego.a_lon_max = (
            self.configuration.vehicle.curvilinear.a_lon_max
        )
        self.reach_config.vehicle.ego.a_lat_min = (
            self.configuration.vehicle.curvilinear.a_lat_min
        )
        self.reach_config.vehicle.ego.a_lat_max = (
            self.configuration.vehicle.curvilinear.a_lat_max
        )
        self.reach_config.planning.dt = self.sce.dt
        # self.reach_config.planning.coordinate_system = "CART"
        self.reach_config.update()
        self.reach_interface = ReachableSetInterface(self.reach_config)

    def _update_initial_state(self, target_state: State):
        self.reach_config.planning_problem.initial_state.position = (
            target_state.position
        )
        self.reach_config.planning_problem.initial_state.velocity = (
            target_state.velocity
        )
        self.reach_config.planning_problem.initial_state.orientation = (
            target_state.orientation
        )
        self.reach_config.planning_problem.initial_state.time_step = (
            target_state.time_step
        )

    def compute(self, time_step: int = 0, vehicle_id=None, verbose: bool = True):
        if not self.validate_update_states_log(vehicle_id, time_step, verbose):
            return np.nan
        self.ttc = self.ttc_object.compute(time_step, verbose=verbose)
        if self.ttc == 0:
            self.value = -math.inf
        elif self.ttc == math.inf:
            self.value = math.inf
        else:
            self.value = self.binary_search(time_step, verbose=verbose)
        if self.value in [math.inf, -math.inf]:
            utils_log.print_and_log_info(
                logger, f"*\t\t {self.measure_name} = {self.value}"
            )
            return self.value
        self.value = utils_gen.int_round(self.value, str(self.dt)[::-1].find("."))
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}"
        )
        return self.value

    def binary_search(self, initial_step: int, verbose=False):
        """
        Binary search to find the last time to execute the maneuver.
        """
        wttr = -math.inf
        low = initial_step
        tstc = int(
            utils_gen.int_round(
                self.ttc / self.dt + self.time_step, str(self.dt)[::-1].find(".")
            )
        )
        high = tstc + initial_step
        time_end = self.ego_vehicle.prediction.final_time_step
        while low < high:
            mid = int((low + high) / 2)
            mid_state = copy.deepcopy(self.ego_vehicle.state_at_time(mid))
            self._update_initial_state(mid_state)

            # update configurations
            self.reach_config.update(
                planning_problem=self.reach_config.planning_problem
            )
            self.reach_config.planning.steps_computation = (
                time_end - mid_state.time_step
            )
            self.reach_config.scenario.remove_obstacle(
                self.reach_config.scenario.obstacle_by_id(self.ego_vehicle.obstacle_id)
            )
            self.reach_config.debug.save_config = verbose

            # reset the interface and compute the reachable sets
            self.reach_interface.reset(self.reach_config)
            self.reach_interface.compute_reachable_sets(verbose=verbose)
            if self.reach_interface.reachable_set_at_step(time_end):
                # the final step is still reachable without causing the collision
                low = mid + 1
            else:
                high = mid
        if low != initial_step:
            # no -1 (differs from the binary search for TTM)
            wttr = (low - initial_step) * self.dt
        return wttr

    def visualize(self):
        if self.value not in [math.inf, -math.inf]:
            wtstr = (
                int(utils_gen.int_round(self.value / self.dt, 0)) + self.time_step - 1
            )
            mid_state = copy.deepcopy(self.ego_vehicle.state_at_time(wtstr))
            self._update_initial_state(mid_state)
            self.reach_config.update(
                planning_problem=self.reach_config.planning_problem
            )
            self.reach_config.scenario.remove_obstacle(
                self.reach_config.scenario.obstacle_by_id(self.ego_vehicle.obstacle_id)
            )
            self.reach_config.planning.steps_computation = (
                self.ego_vehicle.prediction.final_time_step - mid_state.time_step
            )
            self.reach_interface.reset(self.reach_config)

            self.reach_interface.compute_reachable_sets(verbose=False)
            util_visual.plot_scenario_with_reachable_sets(self.reach_interface)
            # util_visual.plot_collision_checker(self.reach_interface)
