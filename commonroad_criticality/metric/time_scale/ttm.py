__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import matplotlib.pyplot as plt
from typing import Union

from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_criticality.data_structure.base import CriticalityBase
from commonroad_criticality.utility.simulation import SimulationLong, SimulationLat, Maneuver
from commonroad_criticality.metric.time_scale.ttc import TTC
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
import commonroad_criticality.utility.visualization as Utils_vis
import commonroad_criticality.utility.general as Utils_gen


class TTM(CriticalityBase):
    metric_name = TimeScaleMetricType.TTM

    def __init__(self,
                 config: CriticalityConfiguration,
                 maneuver: Union[Maneuver, None]):
        super(TTM, self).__init__(config)
        self._maneuver = maneuver
        if maneuver in [Maneuver.BRAKE,
                        Maneuver.KICKDOWN,
                        Maneuver.CONSTANT]:
            self.simulator = SimulationLong(maneuver, self.ego_vehicle, config)
        elif maneuver in [Maneuver.STEERLEFT,
                          Maneuver.STEERRIGHT]:
            self.simulator = SimulationLat(maneuver, self.ego_vehicle, config)
        else:
            self.simulator = None
        self.ttc_object = TTC(config)
        self.ttc = self.ttc_object.compute(rnd=self.rnd)

    @property
    def maneuver(self):
        return self._maneuver

    @maneuver.setter
    def maneuver(self, maneuver: Maneuver):
        self._maneuver = maneuver

    def visualize(self):
        if self.configuration.debug.draw_visualization:
            if self.value not in [math.inf, -math.inf]:
                tstm = int(Utils_gen.int_round(self.value / self.dt, 0))
                Utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(tstm))
                tstc = int(Utils_gen.int_round(self.ttc / self.dt, 0))
                Utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, tstc, 'r')
                Utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(tstc), 'r')
            else:
                tstm = self.value
            plt.title(f"{self.metric_name} at time step {tstm}")
            if self.configuration.debug.save_plots:
                Utils_vis.save_fig(self.metric_name, self.configuration.general.path_output,
                                   tstm)
            else:
                plt.show()

    def compute(self, time_step: int = 0, rnd: MPRenderer = None):
        if self.configuration.debug.draw_visualization:
            self.initialize_vis(time_step, rnd)

        if self.ttc == 0:
            self.value = -math.inf
        elif self.ttc == math.inf:
            self.value = math.inf
        else:
            self.value = self.binary_search(time_step)
        if self.value in [math.inf, -math.inf]:
            return self.value
        return Utils_gen.int_round(self.value, 1)

    def binary_search(self, initial_step: int) -> float:
        """
        Binary search to find the last time to execute the maneuver.
        """
        ttm = - math.inf
        low = initial_step
        high = int(self.ttc / self.dt)
        while low < high:
            mid = int((low + high) / 2)
            state_list = self.simulator.simulate_state_list(mid, self.rnd)
            # flag for successful simulation, 0: False, 1: True
            flag_succ = state_list[-1].time_step == self.ego_vehicle.prediction.final_time_step
            # flag for collision, 0: False, 1: True
            flag_coll = self.ttc_object.detect_collision(state_list)
            if not flag_coll and flag_succ:
                low = mid + 1
            else:
                high = mid
        if low != initial_step:
            ttm = (low - initial_step - 1) * self.dt
        return ttm
