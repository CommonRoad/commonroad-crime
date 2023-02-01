__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import logging

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.type import TypeAcceleration, TypeMonotone
from commonroad_crime.measure.distance.hw import HW
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class ALongReq(CriMeBase):
    """
    The required longitudinal acceleration measures the longitudinal acceleration required to bring the relative
    velocity to zero at the time of the collision.

    - from Sec.5.3.5 in Jansson J, Collision Avoidance Theory: With application to automotive collision mitigation.
    PhD Thesis, 2005, Linköping University, Linköping, Sweden.
    """
    measure_name = TypeAcceleration.ALongReq
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(ALongReq, self).__init__(config)
        self._hw_object = HW(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step
        if self._except_obstacle_in_same_lanelet(expected_value=0.0):
            # no negative acceleration is needed for avoiding a collision
            return self.value
        lanelet_id = self.sce.lanelet_network.find_lanelet_by_position([self.ego_vehicle.state_at_time(time_step).
                                                                       position])[0]
        # orientation of the ego vehicle and the other vehicle
        ego_orientation = utils_sol.compute_lanelet_width_orientation(
            self.sce.lanelet_network.find_lanelet_by_id(lanelet_id[0]),
            self.ego_vehicle.state_at_time(time_step).position
        )[1]
        other_orientation = utils_sol.compute_lanelet_width_orientation(
            self.sce.lanelet_network.find_lanelet_by_id(lanelet_id[0]),
            self.other_vehicle.state_at_time(time_step).position
        )[1]
        # acceleration of the other vehicle along the lanelet
        a_obj = math.sqrt(self.other_vehicle.state_at_time(time_step).acceleration ** 2 +
                          self.other_vehicle.state_at_time(time_step).acceleration_y ** 2) * math.cos(
            other_orientation)
        # compute the headway (relative distance) along the lanelet
        x_rel = self._hw_object.compute(vehicle_id, time_step)
        # compute the vehicles' velocity along the lanelet direction
        v_ego_long = math.sqrt(self.ego_vehicle.state_at_time(time_step).velocity ** 2 + self.ego_vehicle.state_at_time(
            time_step).velocity_y ** 2) * math.cos(ego_orientation)
        v_other_long = math.sqrt(
            self.other_vehicle.state_at_time(time_step).velocity ** 2 + self.other_vehicle.state_at_time(
                time_step).velocity_y ** 2) * math.cos(other_orientation)
        if self.configuration.acceleration.acceleration_mode == 1:
            # constant acceleration using (8) in "Using extreme value theory for vehicle level safety validation and
            # implications for autonomous vehicles." is in correct
            v_rel = v_other_long - v_ego_long
            utils_log.print_and_log_info(logger, f"*\t\t relative velocity is {v_rel}")
            a_req = min(a_obj - v_rel ** 2 / (2 * x_rel), 0.0)
        else:
            # piecewise constant motion using (5.39) in "Collision Avoidance Theory with Application to Automotive
            # Collision Mitigation"
            a_req = - v_ego_long ** 2 / (2 * (x_rel - v_other_long ** 2 / 2 * a_obj))
        if a_req > 0:  # the object is non-closing
            self.value = 0.
        else:
            self.value = utils_gen.int_round(a_req, 2)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self):
        pass
