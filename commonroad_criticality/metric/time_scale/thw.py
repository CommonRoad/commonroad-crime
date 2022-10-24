__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import matplotlib.pyplot as plt

from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_criticality.data_structure.base import CriticalityBase
from commonroad_criticality.data_structure.configuration import CriticalityConfiguration
from commonroad_criticality.data_structure.metric import TimeScaleMetricType
import commonroad_criticality.utility.visualization as Utils_vis
import commonroad_criticality.utility.general as Utils_gen


class THW(CriticalityBase):
    """
    https://criticality-metrics.readthedocs.io/en/latest/time-scale/THW.html
    """
    metric_name = TimeScaleMetricType.THW

    def __init__(self, config: CriticalityConfiguration):
        super(THW, self).__init__(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        self.set_other_vehicles(vehicle_id)
        other_position = self.other_vehicle.state_at_time(time_step).position
        other_s, _ = self.clcs.convert_to_curvilinear_coords(other_position[0], other_position[1])
        self.value = 0.  # at default, we assume that the ego vehicle is already in front
        for ts in range(time_step, self.ego_vehicle.prediction.final_time_step + 1):
            ego_position = self.ego_vehicle.state_at_time(ts).position
            ego_s, _ = self.clcs.convert_to_curvilinear_coords(ego_position[0], ego_position[1])
            if ego_s > other_s:
                self.value = (ts - time_step) * self.dt
                break
        if self.configuration.debug.draw_visualization:
            self.rnd = MPRenderer()
            self.sce.draw(self.rnd, draw_params={'time_begin': time_step,
                                                 "dynamic_obstacle": {
                                                     "draw_icon": self.configuration.debug.draw_icons}})
        return Utils_gen.int_round(self.value, 1)

    def visualize(self):
        if self.configuration.debug.draw_visualization:
            self.rnd.render()
            if self.value > 0:
                tshw = int(Utils_gen.int_round(self.value / self.dt, 0))
                Utils_vis.draw_cut_off_state(self.rnd, self.ego_vehicle.state_at_time(tshw))
            else:
                tshw = self.value
            plt.title(f"{self.metric_name} at time step {tshw}")
            if self.configuration.debug.save_plots:
                Utils_vis.save_fig(self.metric_name, self.configuration.general.path_output,
                                   tshw)
            else:
                plt.show()