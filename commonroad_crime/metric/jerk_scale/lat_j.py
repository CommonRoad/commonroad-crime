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

    def compute(self, time_step: int):
        self.time_step = time_step
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        evaluated_state = self.ego_vehicle.state_at_time(self.time_step)
        self.value = utils_gen.int_round(evaluated_state.jerk * math.sin(evaluated_state.orientation), 2)
        return self.value

    def visualize(self):
        pass