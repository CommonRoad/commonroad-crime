__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math

import matplotlib.pyplot as plt
import logging

import numpy as np

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.type import TypeAccelerationScale
from commonroad_crime.metric.distance_scale.hw import HW
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class ALongReq(CriMeBase):
    """
    The required longitudinal acceleration measures the longitudinal acceleration required to bring the relative
    velocity to zero at the time of the collision.

    - from Sec.5.3.5 in Jansson J, Collision Avoidance Theory: With application to automotive collision mitigation.
    PhD Thesis, 2005, Linköping University, Linköping, Sweden.
    """
    metric_name = TypeAccelerationScale.ALongReq

    def __init__(self, config: CriMeConfiguration):
        super(ALongReq, self).__init__(config)
        self._hw_object = HW(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        self._set_other_vehicles(vehicle_id)
        self.time_step = time_step
        if self._except_obstacle_in_same_lanelet(expected_value=0.0):
            # no negative acceleration is needed for avoiding a collision
            return self.value
        if hasattr(self.other_vehicle.state_at_time(time_step), 'acceleration'):
            a_obj = self.other_vehicle.state_at_time(time_step).acceleration
        elif self.other_vehicle.state_at_time(time_step + 1):
            a_obj = utils_sol.compute_acceleration(self.other_vehicle.state_at_time(time_step).velocity,
                                                   self.other_vehicle.state_at_time(time_step + 1).velocity,
                                                   self.dt)
        else:
            a_obj = 0.
        # compute the headway distance
        x_rel = self._hw_object.compute(vehicle_id, time_step)
        if self.configuration.acceleration_scale.acceleration_mode == 1:
            # constant acceleration using (8) in "Using extreme value theory for vehicle level safety validation and
            # implications for autonomous vehicles."
            v_rel = self.ego_vehicle.state_at_time(time_step).velocity - self.other_vehicle.state_at_time(
                time_step).velocity
            a_req = a_obj - v_rel**2/(2*x_rel)
        else:
            # piecewise constant motion using (5.39) in "Collision Avoidance Theory with Application to Automotive
            # Collision Mitigation"
            a_req = - self.ego_vehicle.state_at_time(time_step).velocity**2 / (
                    2 * (x_rel - self.other_vehicle.state_at_time(time_step).velocity**2/2*a_obj))
        if a_req > 0:  # the object is non-closing
            self.value = 0.
        else:
            self.value = utils_gen.int_round(a_req, 2)
        utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
        return self.value

    def visualize(self):
        pass
