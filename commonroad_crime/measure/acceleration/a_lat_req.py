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
from commonroad_crime.measure.time.ttc import TTC

import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class ALatReq(CriMeBase):
    """
    The required lateral acceleration necessary for collision avoidance at TTC.

    - from Sec.5.3.8 in Jansson J, Collision Avoidance Theory: With application to automotive collision mitigation.
    PhD Thesis, 2005, Linköping University, Linköping, Sweden.
    """
    measure_name = TypeAcceleration.ALatReq
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(ALatReq, self).__init__(config)
        self._ttc_object = TTC(config)

    def _compute_a_lat(self, a_obj_lat: float, d_rel_lat: float, v_rel_lat: float, ttc: float, flag_dir: str) -> float:
        """
        compute the acceleration required to pass to the left or the right of the obstacle

        :param a_obj_lat: lateral acceleration of the other obstacle
        :param d_rel_lat: lateral distance between two obstacles
        :param v_rel_lat: relative velocity between two obstacles along the latteral direction
        :param ttc: time-to-collision
        :param flag_dir: direction - left or right

        :return a_lat: required lateral acceleration of the ego vehicle
        """
        sign = 1 if flag_dir == 'left' else -1
        return a_obj_lat - 2 * (-d_rel_lat + sign * (self.other_vehicle.obstacle_shape.width/2 +
                                                     self.ego_vehicle.obstacle_shape.width/2) - v_rel_lat * ttc)/ttc**2

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
        ttc = self._ttc_object.compute(vehicle_id, time_step)
        utils_log.print_and_log_info(logger, f"*\t\t TTC is equal to {ttc}")
        if ttc == math.inf:
            # no lateral acceleration is needed for avoiding a collision
            self.value = 0.
            return self.value
        a_obj_lat = math.sqrt(self.other_vehicle.state_at_time(time_step).acceleration_y ** 2 +
                              self.other_vehicle.state_at_time(time_step).acceleration ** 2) * \
                    math.sin(other_orientation)

        # compute the headway distance
        d_rel_lat = utils_sol.compute_clcs_distance(self.clcs,
                                                    self.ego_vehicle.state_at_time(time_step).position,
                                                    self.ego_vehicle.state_at_time(time_step).position)[1]
        v_rel_lat = (math.sqrt(self.other_vehicle.state_at_time(time_step).velocity ** 2 +
                               self.other_vehicle.state_at_time(time_step).velocity_y ** 2) * math.sin(
            other_orientation) -
                     math.sqrt(self.ego_vehicle.state_at_time(time_step).velocity ** 2 +
                               self.ego_vehicle.state_at_time(time_step).velocity_y ** 2)) * math.sin(ego_orientation)
        self.value = min(
            abs(self._compute_a_lat(a_obj_lat, d_rel_lat, v_rel_lat, ttc, 'left')),
            abs(self._compute_a_lat(a_obj_lat, d_rel_lat, v_rel_lat, ttc, 'right'))
        )
        if self.value != math.inf:
            self.value = utils_gen.int_round(self.value, 2)

        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self):
        pass
