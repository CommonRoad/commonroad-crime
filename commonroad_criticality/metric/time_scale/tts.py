__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.base import CriticalityBase
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
from commonroad_criticality.metric.time_scale.ttm import TTM
from commonroad_criticality.utility.simulation import Maneuver
import commonroad_criticality.utility.visualization as Utils_vis

from commonroad.visualization.mp_renderer import MPRenderer


class TTS(CriticalityBase):
    metric_name = TimeScaleMetricType.TTS

    def __init__(self, config: CriticalityConfiguration):
        super(TTS, self).__init__(config)
        self._left_evaluator = TTM(config, Maneuver.STEERLEFT)
        self._right_evaluator = TTM(config, Maneuver.STEERRIGHT)
        self._left_evaluator.metric_name = self.metric_name
        self._right_evaluator.metric_name = self.metric_name
        self.maneuver = Maneuver.NONE

    def compute(self, time_step: int = 0, rnd: MPRenderer = None):
        if rnd:
            self.rnd = rnd
        else:
            self.rnd = MPRenderer()
            Utils_vis.draw_sce_at_time_step(self.rnd, self.configuration, self.sce, time_step)
        tts_left = self._left_evaluator.compute(time_step, self.rnd)
        tts_right = self._right_evaluator.compute(time_step, self.rnd)
        if tts_left > tts_right:
            self.maneuver = Maneuver.STEERLEFT
        else:
            self.maneuver = Maneuver.STEERRIGHT
        self.value = max(tts_left, tts_right)
        return self.value

    def visualize(self):
        if self.maneuver is Maneuver.STEERLEFT:
            self._left_evaluator.visualize()
        else:
            self._right_evaluator.visualize()
