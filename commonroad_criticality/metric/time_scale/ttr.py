__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

from commonroad_criticality.metric.time_scale.ttb import TTB
from commonroad_criticality.metric.time_scale.ttk import TTK
from commonroad_criticality.metric.time_scale.tts import TTS
from commonroad_criticality.metric.time_scale.ttm import TTM
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
from commonroad_criticality.utility.simulation import Maneuver

from commonroad.visualization.mp_renderer import MPRenderer


class TTR(TTM):
    metric_name = TimeScaleMetricType.TTR

    def __init__(self, config: CriticalityConfiguration):
        super(TTR, self).__init__(config, Maneuver.NONE)
        self._evaluator = [TTB(config), TTK(config), TTS(config)]

    def compute(self, time_step: int = 0, rnd: MPRenderer = None):
        if self.configuration.debug.draw_visualization:
            self.rnd = MPRenderer()
            self.sce.draw(self.rnd, draw_params={'time_begin': time_step,
                                                 "dynamic_obstacle": {
                                                     "draw_icon": self.configuration.debug.draw_icons}})
            self.rnd.render()
        ttm = dict()
        for evl in self._evaluator:
            ttm[evl.maneuver] = evl.compute(time_step, rnd=self.rnd)
        self.value = max(ttm.values())
        self.maneuver = max(ttm, key=ttm.get)
        return self.value

