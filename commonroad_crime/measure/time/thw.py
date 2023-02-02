__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import matplotlib.pyplot as plt
import logging
import math

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class THW(CriMeBase):
    """
    https://criticality-metrics.readthedocs.io/en/latest/time-scale/THW.html
    """
    measure_name = TypeTime.THW
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(THW, self).__init__(config)

    def cal_headway(self):
        other_position = self.other_vehicle.state_at_time(self.time_step).position
        other_s, _ = self.clcs.convert_to_curvilinear_coords(other_position[0], other_position[1])
        ego_position = self.ego_vehicle.state_at_time(self.time_step).position
        ego_s, _ = self.clcs.convert_to_curvilinear_coords(ego_position[0], ego_position[1])

        # bump position
        ego_s += self.ego_vehicle.obstacle_shape.length/2
        other_s -= self.other_vehicle.obstacle_shape.length/2

        if ego_s > other_s:
            return math.inf
        for ts in range(self.time_step + 1, self.ego_vehicle.prediction.final_time_step + 1):
            ego_position = self.ego_vehicle.state_at_time(ts).position
            ego_s, _ = self.clcs.convert_to_curvilinear_coords(ego_position[0], ego_position[1])
            if ego_s > other_s:
                return utils_gen.int_round((ts - self.time_step) * self.dt,
                                           str(self.dt)[::-1].find('.'))
        return math.inf

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}", verbose)
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step

        if self._except_obstacle_in_same_lanelet(expected_value=math.inf):
            return self.value
        self.value = self.cal_headway()
        if self.value is not math.inf:
            self.value = utils_gen.int_round(self.value, 2)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} with vehicle id {vehicle_id} = {self.value}")
        return self.value

    def visualize(self, figsize: tuple = (25, 15)):
        self._initialize_vis(figsize=figsize,
                             plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                             self.ego_vehicle.prediction.trajectory.state_list,
                                                                             margin=10))
        self.rnd.render()
        utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                  color=TUMcolor.TUMblue, linewidth=5)
        if self.value > 0 and self.value is not math.inf:
            print(self.value)

            tshw = int(utils_gen.int_round(self.value / self.dt, 0))
            utils_vis.draw_state(self.rnd, self.ego_vehicle.state_at_time(tshw + self.time_step))
            utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, tshw + self.time_step)
        else:
            tshw = self.value
        plt.title(f"{self.measure_name} at time step {tshw}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output,
                               tshw)
        else:
            plt.show()
