__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import numpy as np
import matplotlib.pyplot as plt
import logging

from commonroad.scenario.obstacle import StaticObstacle
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class WTTC(CriMeBase):
    """
    The worst-time-to-collision metric extends the usual TTC by considering multiple traces of actors as predicted by an
    over-approximating dynamic motion model. From: Wachenfeld, Walther, et al. "The worst-time-to-collision metric
    for situation identification." 2016 IEEE Intelligent Vehicles Symposium (IV). IEEE, 2016.
    """
    measure_name = TypeTime.WTTC
    monotone = TypeMonotone.NEG
    
    def __init__(self, config: CriMeConfiguration):
        super(WTTC, self).__init__(config)

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        self.time_step = time_step
        self.set_other_vehicles(vehicle_id)
        wttc_list = utils_sol.solver_wttc(self.ego_vehicle,
                                          self.other_vehicle,
                                          time_step,
                                          self.configuration.vehicle.cartesian.longitudinal.a_max)
        self.value = max([np.real(x) for x in wttc_list if np.isreal(x) and x > 0][0], 0.0)
        self.value = utils_gen.int_round(self.value, str(self.dt)[::-1].find('.'))
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self, figsize: tuple = (25, 15)):
        self._initialize_vis(figsize=figsize,
                             plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                              self.ego_vehicle.prediction.
                                                                              trajectory.state_list,
                                                                              margin=10))
        self.rnd.render()
        if self.time_step == 0 and self.ego_vehicle.prediction.trajectory.state_list[0].time_step != 0:
            utils_vis.draw_state_list(self.rnd, [self.ego_vehicle.initial_state] +
                                      self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                      color=TUMcolor.TUMblue, linewidth=5)
        else:
            utils_vis.draw_state_list(self.rnd, self.ego_vehicle.prediction.trajectory.state_list[self.time_step:],
                                      color=TUMcolor.TUMblue, linewidth=5)
        r_1 = r_2 = 1/2 * self.configuration.vehicle.cartesian.longitudinal.a_max * self.value ** 2
        if isinstance(self.other_vehicle, StaticObstacle):
            r_2 = 0
        r_v1, _ = utils_sol.compute_disc_radius_and_distance(self.ego_vehicle.obstacle_shape.length,
                                                             self.ego_vehicle.obstacle_shape.width)
        r_v2, _ = utils_sol.compute_disc_radius_and_distance(self.other_vehicle.obstacle_shape.length,
                                                             self.other_vehicle.obstacle_shape.width)
        wtstc = int(utils_gen.int_round(self.value / self.dt, 0))
        new_x_1 = self.ego_vehicle.state_at_time(self.time_step).position[0] + \
                  self.value * self.ego_vehicle.state_at_time(self.time_step).velocity
        new_y_1 = self.ego_vehicle.state_at_time(self.time_step).position[1] + \
                  self.value * self.ego_vehicle.state_at_time(self.time_step).velocity_y
        new_x_2 = self.other_vehicle.state_at_time(self.time_step).position[0] + \
                  self.value * self.other_vehicle.state_at_time(self.time_step).velocity
        new_y_2 = self.other_vehicle.state_at_time(self.time_step).position[1] + \
                  self.value * self.other_vehicle.state_at_time(self.time_step).velocity_y
        utils_vis.draw_dyn_vehicle_shape(self.rnd, self.ego_vehicle, self.time_step)
        utils_vis.draw_circle(self.rnd, np.array([new_x_1, new_y_1]),
                              r_v1 + r_1, color=TUMcolor.TUMblue)
        utils_vis.draw_circle(self.rnd, np.array([new_x_1, new_y_1]),
                              r_v1, color=TUMcolor.TUMdarkblue)
        utils_vis.draw_circle(self.rnd, np.array([new_x_2, new_y_2]),
                              r_v2 + r_2, color=TUMcolor.TUMlightgray)
        utils_vis.draw_circle(self.rnd, np.array([new_x_2, new_y_2]),
                              r_v2, color=TUMcolor.TUMdarkgray)
        plt.title(f"{self.measure_name} at time step {self.time_step}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output,
                               self.time_step)
        else:
            plt.show()



