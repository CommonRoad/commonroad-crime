__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.4.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import logging
import numpy as np

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeIndex, TypeMonotone
from commonroad_crime.measure.acceleration.a_lat_req import ALatReq
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class STN(CriMeBase):
    """
    the ratio between the requested lateral acceleration (due to steering) and the maximum achievable lateral
    acceleration is denoted as the steering threat number (STN).

    -- from Hosseini, SeyedMehrdad, et al. "Adaptive forward collision warning algorithm for automotive applications."
    2016 American Control Conference (ACC). IEEE, 2016.
    """

    measure_name = TypeIndex.STN
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(STN, self).__init__(config)
        self._a_lat_req_object = ALatReq(config)

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        if not self.validate_update_states_log(vehicle_id, time_step, verbose):
            return np.nan
        a_lat_req = self._a_lat_req_object.compute(
            vehicle_id, time_step, verbose=verbose
        )
        # (1) in "Adaptive forward collision warning algorithm for automotive applications."
        self.value = utils_gen.int_round(
            abs(a_lat_req / self.configuration.vehicle.curvilinear.a_lat_max), 4
        )
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}", verbose
        )
        return self.value

    def visualize(self):
        pass
