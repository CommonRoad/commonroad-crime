__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging

from commonroad_criticality.metric.time_scale.ttb import TTB
from commonroad_criticality.metric.time_scale.ttk import TTK
from commonroad_criticality.metric.time_scale.tts import TTS
from commonroad_criticality.metric.time_scale.ttm import TTM
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
from commonroad_criticality.utility.simulation import Maneuver
import commonroad_criticality.utility.logger as utils_log

from commonroad.visualization.mp_renderer import MPRenderer

logger = logging.getLogger(__name__)


class TTR(TTM):
    metric_name = TimeScaleMetricType.TTR

    def __init__(self, config: CriticalityConfiguration):
        super(TTR, self).__init__(config, Maneuver.NONE)
        self._evaluator = [TTB(config), TTK(config), TTS(config)]

    def compute(self, time_step: int = 0, ttc: float = None, rnd: MPRenderer = None, verbose: bool = False):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}", verbose)
        self.ttc = self.ttc_object.compute(time_step, rnd)
        if self.configuration.debug.draw_visualization:
            self.initialize_vis(time_step, rnd)
        ttm = dict()
        for evl in self._evaluator:
            ttm[evl.maneuver] = evl.compute(time_step, self.ttc, rnd=self.rnd, verbose=verbose)
        self.value = max(ttm.values())
        self.maneuver = max(ttm, key=ttm.get)
        utils_log.print_and_log_info(logger, "*\t maximum of the values")
        utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
        return self.value

