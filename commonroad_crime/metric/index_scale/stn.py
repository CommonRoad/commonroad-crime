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
from commonroad_crime.data_structure.type import TypeIndexScale
from commonroad_crime.metric.distance_scale.hw import HW
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.visualization as utils_vis

logger = logging.getLogger(__name__)


class STN(CriMeBase):
    """
    the ratio between the requested lateral acceleration (due to steering) and the maximum achievable lateral
    acceleration.

    -- from Hosseini S, Murgovski N, De Campos GR, Sjöberg J. Adaptive forward collision warning algorithm for
    automotive applications. In2016 American Control Conference (ACC) 2016 Jul 6 (pp. 5982-5987). IEEE.
    """
    metric_name = TypeIndexScale.STN

    def __init__(self, config: CriMeConfiguration):
        super(STN, self).__init__(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        self._set_other_vehicles(vehicle_id)
        self.time_step = time_step


    def visualize(self):
        pass