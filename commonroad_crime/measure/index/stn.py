__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging
import matplotlib.pyplot as plt

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeIndex, TypeMonotone
from commonroad_crime.measure.acceleration.a_lat_req import ALatReq
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.visualization as utils_vis

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

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step
        a_lat_req = self._a_lat_req_object.compute(vehicle_id, time_step)
        # (1) in "Adaptive forward collision warning algorithm for automotive applications."
        self.value = utils_gen.int_round(abs(a_lat_req / self.configuration.vehicle.curvilinear.a_lat_max), 4)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self):
        pass