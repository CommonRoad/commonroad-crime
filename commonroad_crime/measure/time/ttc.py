__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging
import math

import matplotlib.pyplot as plt
import numpy as np
from commonroad.scenario.obstacle import DynamicObstacle

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.visualization as utils_vis

from commonroad_crime.measure.distance.hw import HW

logger = logging.getLogger(__name__)


class TTC(CriMeBase):
    """
    With a constant acceleration decision model of the vehicles motion

    -- using (5.24) in "Collision Avoidance Theory with Application to Automotive Collision Mitigation"
    """
    measure_name = TypeTime.TTC
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(TTC, self).__init__(config)
        self._hw_object = HW(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step
        state = self.ego_vehicle.state_at_time(time_step)
        state_other = self.other_vehicle.state_at_time(time_step)
        lanelet_id = self.sce.lanelet_network.find_lanelet_by_position([self.ego_vehicle.state_at_time(time_step).
                                                                       position])[0]
        """
        Using https://www.diva-portal.org/smash/get/diva2:617438/FULLTEXT01.pdf 
        "Collision Avoidance Theory with Application to Automotive Collision Mitigation" formula 5.26
        """

        # distance along the lanelet
        delta_d = self._hw_object.compute(vehicle_id, time_step)

        if delta_d == math.inf:
            self.value = math.inf
        else:
            # orientation of the ego vehicle and the other vehicle
            ego_orientation = utils_sol.compute_lanelet_width_orientation(
                self.sce.lanelet_network.find_lanelet_by_id(lanelet_id[0]),
                self.ego_vehicle.state_at_time(time_step).position
            )[1]
            other_orientation = utils_sol.compute_lanelet_width_orientation(
                self.sce.lanelet_network.find_lanelet_by_id(lanelet_id[0]),
                self.other_vehicle.state_at_time(time_step).position
            )[1]
            # actual velocity and acceleration of both vehicles along the lanelet
            v_ego = np.sign(state.velocity) * math.sqrt(state.velocity ** 2 + state.velocity_y ** 2) * math.cos(
                ego_orientation)
            # include the directions
            a_ego = np.sign(state.acceleration) * math.sqrt(state.acceleration ** 2 + state.acceleration_y ** 2) * math.cos(
                ego_orientation)
            if isinstance(self.other_vehicle, DynamicObstacle):
                v_other = np.sign(state_other.velocity) * math.sqrt(
                    state_other.velocity ** 2 + state_other.velocity_y ** 2) * math.cos(other_orientation)
                a_other = np.sign(state_other.acceleration) * math.sqrt(
                    state_other.acceleration ** 2 + state_other.acceleration_y ** 2) * math.cos(other_orientation)
            else:
                v_other = 0
                a_other = 0
            delta_v = v_other - v_ego
            delta_a = a_other - a_ego

            if delta_v < 0 and abs(delta_a) <= 0.1:
                self.value = utils_gen.int_round(- (delta_d / delta_v), 2)
            elif np.sqrt(delta_v ** 2 - 2 * delta_d * delta_a) / delta_a < 0:
                self.value = math.inf
            elif (delta_v < 0 and delta_a != 0) or (delta_v >= 0 and delta_a < 0):
                first = - (delta_v / delta_a)
                second = np.sqrt(delta_v ** 2 - 2 * delta_d * delta_a) / delta_a
                if delta_v < 0:
                    self.value = utils_gen.int_round(first - second, 2)
                else:
                    self.value = utils_gen.int_round(first + second, 2)
            else:  # (delta_v >= 0 and delta_a >= 0) or (delta_v ** 2 - 2 * delta_d * delta_a < 0)
                self.value = math.inf

        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self):
        self._initialize_vis(plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                              self.ego_vehicle.prediction.
                                                                              trajectory.state_list,
                                                                              margin=10))
        self.other_vehicle.draw(self.rnd, {'time_begin': self.time_step, **utils_vis.OTHER_VEHICLE_DRAW_PARAMS})
        self.rnd.render()
        plt.title(f"{self.measure_name} at time step {self.time_step}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()
