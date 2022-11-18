__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import logging

from commonroad.scenario.scenario import State

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeJerkScale
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol

logger = logging.getLogger(__name__)


class LatJ(CriMeBase):
    """
    Jerk is the rate of change in acceleration, and thus quantifies over the abruptness of a maneuver.
    """
    metric_name = TypeJerkScale.LatJ

    def __init__(self, config: CriMeConfiguration):
        super(LatJ, self).__init__(config)

    def _compute_jerk(self, evaluated_state: State):
        if hasattr(evaluated_state, 'jerk'):
            return evaluated_state.jerk
        else:
            if hasattr(evaluated_state, 'acceleration'):
                current_acc = evaluated_state.acceleration
            elif self.ego_vehicle.state_at_time(self.time_step + 1):
                current_acc = utils_sol.compute_acceleration(
                    evaluated_state.velocity,
                    self.ego_vehicle.state_at_time(self.time_step + 1).velocity,
                    self.dt
                )
            else:
                current_acc = None
            if self.ego_vehicle.state_at_time(self.time_step + 1):
                if hasattr(self.ego_vehicle.state_at_time(self.time_step + 1), 'acceleration'):
                    next_acc = self.ego_vehicle.state_at_time(self.time_step + 1).acceleration
                elif self.ego_vehicle.state_at_time(self.time_step + 2):
                    next_acc = utils_sol.compute_acceleration(
                        self.ego_vehicle.state_at_time(self.time_step + 1).velocity,
                        self.ego_vehicle.state_at_time(self.time_step + 1).velocity,
                        self.dt
                    )
                else:
                    next_acc = None
            else:
                next_acc = None
            if current_acc is not None and next_acc is not None:
                return utils_sol.compute_jerk(
                    current_acc,
                    next_acc,
                    self.dt)
            else:
                return None

    def compute(self, time_step: int):
        self.time_step = time_step
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        evaluated_state = self.ego_vehicle.state_at_time(self.time_step)
        jerk = self._compute_jerk(evaluated_state)
        if jerk is not None:
            self.value = utils_gen.int_round(jerk * math.sin(evaluated_state.orientation), 2)
            utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
        else:
            self.value = None
            utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} is none, but you can interpolate it using "
                                                 f"the values from previous time steps")
        return self.value

    def visualize(self):
        pass
