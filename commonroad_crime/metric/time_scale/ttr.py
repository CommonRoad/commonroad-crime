__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging

from commonroad_crime.metric.time_scale.ttb import TTB
from commonroad_crime.metric.time_scale.ttk import TTK
from commonroad_crime.metric.time_scale.tts import TTS
from commonroad_crime.metric.time_scale.ttm import TTM
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.metric import TimeScaleMetricType
from commonroad_crime.utility.simulation import Maneuver
import commonroad_crime.utility.logger as utils_log

from commonroad.visualization.mp_renderer import MPRenderer

logger = logging.getLogger(__name__)


class TTR(TTM):
    metric_name = TimeScaleMetricType.TTR

    def __init__(self, config: CriMeConfiguration):
        super(TTR, self).__init__(config, Maneuver.NONE)
        self._evaluator = None

    def initialize_evaluator(self, time_step):
        self._evaluator = [TTB(self.configuration),
                           TTK(self.configuration),
                           TTS(self.configuration)]
        self.time_step = time_step
        self.state_list_set = []
        self.ttc = self.ttc_object.compute(time_step)

    def compute(self, time_step: int = 0, ttc: float = None, verbose: bool = False):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}", verbose)
        self.initialize_evaluator(time_step)
        ttm = dict()
        for evl in self._evaluator:
            ttm[evl] = evl.compute(time_step, self.ttc, verbose=verbose)
            self.state_list_set += evl.state_list_set
        self.value = max(ttm.values())
        self.selected_state_list = max(ttm, key=ttm.get).selected_state_list
        self.maneuver = max(ttm, key=ttm.get).maneuver
        utils_log.print_and_log_info(logger, "*\t maximum of the values")
        utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
        return self.value
