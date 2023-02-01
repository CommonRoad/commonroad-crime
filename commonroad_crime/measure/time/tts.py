__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.type import TypeTime
from commonroad_crime.measure.time.ttm import TTM
from commonroad_crime.utility.simulation import Maneuver
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class TTS(CriMeBase):
    measure_name = TypeTime.TTS

    def __init__(self, config: CriMeConfiguration):
        super(TTS, self).__init__(config)
        # steer to the left or to the right
        self._left_evaluator = TTM(config, Maneuver.STEERLEFT)
        self._right_evaluator = TTM(config, Maneuver.STEERRIGHT)
        self._left_evaluator.metric_name = self._right_evaluator.metric_name = self.measure_name
        self.maneuver = Maneuver.NONE
        self.selected_state_list = None
        self.state_list_set = []

    def compute(self, time_step: int = 0, vehicle_id: int = None, ttc: float = None, verbose: bool = False):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")

        tts_left = self._left_evaluator.compute(time_step, ttc, verbose=False)
        self.state_list_set += self._left_evaluator.state_list_set

        tts_right = self._right_evaluator.compute(time_step, ttc, verbose=False)
        self.state_list_set += self._right_evaluator.state_list_set

        # decide the specific maneuver for steering
        if tts_left > tts_right:
            self.maneuver = Maneuver.STEERLEFT
            self.selected_state_list = self._left_evaluator.selected_state_list
        else:
            self.maneuver = Maneuver.STEERRIGHT
            self.selected_state_list = self._right_evaluator.selected_state_list
        self.value = max(tts_left, tts_right)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self):
        if self.maneuver is Maneuver.STEERLEFT:
            self._left_evaluator.visualize()
        else:
            self._right_evaluator.visualize()
