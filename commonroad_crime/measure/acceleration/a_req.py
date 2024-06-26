__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.4.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import math
import logging
import numpy as np

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.type import TypeAcceleration, TypeMonotone
from commonroad_crime.measure.acceleration.a_lat_req import ALatReq
from commonroad_crime.measure.acceleration.a_long_req import ALongReq
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class AReq(CriMeBase):
    """
    The required acceleration based on the longitudinal and lateral values.

    from Sec.5.3.10 in Jansson J, Collision Avoidance Theory: With application to automotive collision mitigation.
    PhD Thesis, 2005, Linköping University, Linköping, Sweden.
    """

    measure_name = TypeAcceleration.AReq
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(AReq, self).__init__(config)
        self._a_long_object = ALongReq(config)
        self._a_lat_object = ALatReq(config)

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        if not self.validate_update_states_log(vehicle_id, time_step, verbose):
            return np.nan
        self.value = math.sqrt(
            self._a_long_object.compute(vehicle_id, time_step, verbose) ** 2
            + self._a_lat_object.compute(vehicle_id, time_step, verbose) ** 2
        )
        self.value = utils_gen.int_round(self.value, 2)
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}", verbose
        )
        return self.value

    def visualize(self):
        pass
