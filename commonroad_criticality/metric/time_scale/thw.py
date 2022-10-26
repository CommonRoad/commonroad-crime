__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import matplotlib.pyplot as plt
import logging

from commonroad_criticality.data_structure.base import CriticalityBase
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
import commonroad_criticality.utility.visualization as utils_vis
import commonroad_criticality.utility.general as utils_gen
import commonroad_criticality.utility.logger as utils_log

logger = logging.getLogger(__name__)


class THW(CriticalityBase):
    """
    https://criticality-metrics.readthedocs.io/en/latest/time-scale/THW.html
    """
    metric_name = TimeScaleMetricType.THW

    def __init__(self, config: CriticalityConfiguration):
        super(THW, self).__init__(config)
        self._thw_ts = None  # the absolute thw

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}", verbose)

        if self.configuration.debug.draw_visualization:
            self.initialize_vis(time_step, None)
        self.set_other_vehicles(vehicle_id)
        other_position = self.other_vehicle.state_at_time(time_step).position
        other_s, _ = self.clcs.convert_to_curvilinear_coords(other_position[0], other_position[1])
        self.value = 0.  # at default, we assume that the ego vehicle is already in front
        for ts in range(time_step, self.ego_vehicle.prediction.final_time_step + 1):
            ego_position = self.ego_vehicle.state_at_time(ts).position
            ego_s, _ = self.clcs.convert_to_curvilinear_coords(ego_position[0], ego_position[1])
            if ego_s > other_s:
                self._thw_ts = ts
                self.value = (self._thw_ts - time_step) * self.dt
                break
        self.value = utils_gen.int_round(self.value, str(self.dt)[::-1].find('.'))
        utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
        return self.value

    def visualize(self):
        if self.configuration.debug.draw_visualization:
            if self.value > 0:
                tshw = int(utils_gen.int_round(self.value / self.dt, 0))
                utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(self._thw_ts),
                                     self.configuration.debug.save_plots)
                utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, self._thw_ts, 'g')
            else:
                tshw = self.value
            plt.title(f"{self.metric_name} at time step {tshw}")
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.metric_name, self.configuration.general.path_output,
                                   tshw)
            else:
                plt.show()
