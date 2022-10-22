__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
from typing import Union

from commonroad.visualization.mp_renderer import MPRenderer


from commonroad_criticality.data_structure.base import CriticalityBase
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
from commonroad_criticality.utility.simulation import SimulationLong, SimulationLat, Maneuver
from commonroad_criticality.metric.time_scale.ttc import TTC


class TTM(CriticalityBase):
    metric_name = TimeScaleMetricType.TTM

    def __init__(self,
                 config: CriticalityConfiguration,
                 simulator: Union[SimulationLong, SimulationLat]):
        super(TTM, self).__init__(config)
        self.simulator = simulator
        self.ttc_object = TTC(config)
        self.ttc = self.ttc_object.compute()
        self.rnd = MPRenderer()

    def compute(self):
        pass

    def binary_search(self):
        """
        Binary search to find the last time to execute the maneuver.
        """
        ttm = - math.inf
        low = 0
        high = int(self.ttc/self.dt)
        while low < high:
            mid = int((low + high)/2)
            state_list = self.simulator.simulate_state_list(mid, self.rnd)
            # flag for successful simulation, 0: False, 1: True
            flag_succ = state_list[-1].time_step == self.ego_vehicle.prediction.final_time_step
            # flag for collision, 0: False, 1: True
            flag_coll = self.ttc_object.detect_collision(state_list)
            if not flag_coll and flag_succ:
                low = mid + 1
            else:
                high = mid
        if low != 0:
            ttm = (low - 1) * self.dt
        return ttm
    
