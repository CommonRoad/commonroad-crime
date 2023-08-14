__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import logging

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
import commonroad_crime.utility.logger as utils_log


logger = logging.getLogger(__name__)


class TV(CriMeBase):
    """
    See https://criticality-metrics.readthedocs.io/
    """

    measure_name = TypeTime.TV
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(TV, self).__init__(config)

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(logger, "*\t\t coming soon", verbose)

    def visualize(self):
        pass
