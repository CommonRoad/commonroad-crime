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
import logging

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.utility.simulation import SimulationLong, SimulationLat, Maneuver
from commonroad_crime.metric.time_scale.ttc import TTC
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.metric import TimeScaleMetricType
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class TTM(CriMeBase):
    metric_name = TimeScaleMetricType.TTM

    def __init__(self,
                 config: CriMeConfiguration,
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
        self.ttc = None
        self.selected_state_list = None
        self.state_list_set = []

    @property
    def maneuver(self):
        return self._maneuver

    @maneuver.setter
    def maneuver(self, maneuver: Maneuver):
        self._maneuver = maneuver

    def visualize(self, figsize: tuple = (25, 15)):
        self.initialize_vis(figsize=figsize,
                            plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                             self.selected_state_list,
                                                                             margin=10))
        self.ttc_object.draw_collision_checker(self.rnd)
        self.rnd.render()
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMblue, linewidth=5)
        for sl in self.state_list_set:
            utils_vis.draw_state_list(self.rnd, sl)
        if self.value not in [math.inf, -math.inf] and self.ttc:
            tstm = int(utils_gen.int_round(self.value / self.dt, 0)) + self.time_step
            utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(tstm), TUMcolor.TUMgreen)
            tstc = int(utils_gen.int_round(self.ttc / self.dt, 0)) + self.time_step
            utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, tstc)
            # utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(tstc), TUMcolor.TUMred)
            # utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[tstc:],
            #                           color=TUMcolor.TUMred, linewidth=5)
            utils_vis.draw_state_list(self.rnd, self.selected_state_list[tstm:],
                                      color=TUMcolor.TUMgreen, linewidth=5)
        else:
            tstm = self.value

        plt.title(f"{self.metric_name} at time step {tstm - self.time_step}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.metric_name, self.configuration.general.path_output,
                               tstm - self.time_step)
        else:
            plt.show()

    def compute(self, time_step: int = 0, ttc: float = None, verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}", verbose)
        self.time_step = time_step
        if ttc:
            self.ttc = ttc
        else:
            self.ttc = self.ttc_object.compute(time_step)
        if self.ttc == 0:
            self.value = -math.inf
        elif self.ttc == math.inf:
            self.value = math.inf
        else:
            self.value = self.binary_search(time_step)
        if self.value in [math.inf, -math.inf]:
            utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
            return self.value
        self.value = utils_gen.int_round(self.value, str(self.dt)[::-1].find('.'))
        utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
        return self.value

    def binary_search(self, initial_step: int) -> float:
        """
        Binary search to find the last time to execute the maneuver.
        """
        ttm = - math.inf
        low = initial_step
        high = int(utils_gen.int_round(self.ttc / self.dt,  str(self.dt)[::-1].find('.'))) + initial_step
        while low < high:
            mid = int((low + high) / 2)
            state_list = self.simulator.simulate_state_list(mid, self.rnd)
            self.state_list_set.append(state_list[mid:])
            # flag for successful simulation, 0: False, 1: True
            flag_succ = state_list[-1].time_step == self.ego_vehicle.prediction.final_time_step
            # flag for collision, 0: False, 1: True
            flag_coll = self.ttc_object.detect_collision(state_list)
            if not flag_coll and flag_succ:
                low = mid + 1
                self.selected_state_list = state_list
            else:
                high = mid
        if low != initial_step:
            ttm = (low - initial_step - 1) * self.dt
        return ttm