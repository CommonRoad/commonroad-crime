__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import logging

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeJerk, TypeMonotone
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol

logger = logging.getLogger(__name__)


class LatJ(CriMeBase):
    """
    Jerk is the rate of change in acceleration, and thus quantifies over the abruptness of a maneuver.
    """
    measure_name = TypeJerk.LatJ
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(LatJ, self).__init__(config)

    def compute(self, time_step: int, vehicle_id=None):
        self.time_step = time_step
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        lanelet_id = self.sce.lanelet_network.find_lanelet_by_position([self.ego_vehicle.state_at_time(time_step).
                                                                       position])[0]
        # orientation of the ego vehicle and the other vehicle
        ego_orientation = utils_sol.compute_lanelet_width_orientation(
            self.sce.lanelet_network.find_lanelet_by_id(lanelet_id[0]),
            self.ego_vehicle.state_at_time(time_step).position
        )[1]
        evaluated_state = self.ego_vehicle.state_at_time(self.time_step)
        self.value = utils_gen.int_round(abs(evaluated_state.jerk * math.sin(ego_orientation)), 2)
        return self.value

    def visualize(self):
        pass
