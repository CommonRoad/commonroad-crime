__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import numpy as np
from typing import Union
import logging

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeVelocity, TypeMonotone
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class DeltaV(CriMeBase):
    """
    Delta-v is a measures of the change in velocity that a vehicle experiences as a result of a collision

    Shelby, Steven G. "Delta-V as a measures of traffic conflict severity." 3rd International Conference on Road
    Safety and Simulati. September. 2011.
    """
    measure_name = TypeVelocity.Delta_V
    monotone = TypeMonotone.POS

    def __init__(self,
                 config: CriMeConfiguration):
        super(DeltaV, self).__init__(config)

    def compute(self, time_step: int, vehicle_id: Union[int, None]):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step
        m_ego = self.configuration.vehicle.dynamic.parameters.m
        if self.configuration.velocity.m_b:
            m_b = self.configuration.velocity.m_b
        else:
            m_b = m_ego
        if self.ego_vehicle.state_at_time(self.time_step) and self.other_vehicle.state_at_time(self.time_step):
            v_ego = np.sqrt(self.ego_vehicle.state_at_time(self.time_step).velocity ** 2 +
                            self.ego_vehicle.state_at_time(self.time_step).velocity_y ** 2)
            v_b = np.sqrt(self.other_vehicle.state_at_time(self.time_step).velocity ** 2 +
                          self.other_vehicle.state_at_time(self.time_step).velocity_y ** 2)
            delta_v = m_b * (v_ego + v_b * np.cos(self.ego_vehicle.state_at_time(self.time_step).orientation -
                                                  self.other_vehicle.state_at_time(self.time_step).orientation)) / (
                                  m_b + m_ego)
            self.value = utils_gen.int_round(delta_v, 2)
        else:
            utils_log.print_and_log_warning(logger, f"\t the vehicles have no state at time step {self.time_step}")
            self.value = None
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self):
        pass
