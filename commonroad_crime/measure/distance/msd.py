__author__ = "Yuanfei Lin, Ziqian Xu"
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

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeDistance, TypeMonotone
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.solver as utils_sol


logger = logging.getLogger(__name__)


class MSD(CriMeBase):
    """
    See https://criticality-metrics.readthedocs.io/
    """
    measure_name = TypeDistance.MSD
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(MSD, self).__init__(config)

    def compute(self, vehicle_id: int = None, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.time_step = time_step
        state = self.ego_vehicle.state_at_time(time_step)
        lanelet_id = self.sce.lanelet_network.find_lanelet_by_position([self.ego_vehicle.state_at_time(time_step).
                                                                       position])[0]

        # compute the orientation of ego-vehicle
        ego_orientation = utils_sol.compute_lanelet_width_orientation(
                self.sce.lanelet_network.find_lanelet_by_id(lanelet_id[0]),
                self.ego_vehicle.state_at_time(time_step).position
            )[1]
        
        # actual velocity and acceleration of ego-vehicle along the lanelet
        v_ego = np.sign(state.velocity) * math.sqrt(state.velocity ** 2 + state.velocity_y ** 2) * math.cos(
                ego_orientation)
        a_ego = np.sign(state.acceleration) * math.sqrt(
                state.acceleration ** 2 + state.acceleration_y ** 2) * math.cos(
                ego_orientation)
        
        # compute MSD
        if a_ego == 0.:
            self.value = math.inf
        else:
            self.value = utils_gen.int_round(v_ego ** 2 / (2 * np.abs(a_ego)), 2)
        
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self):
        pass
