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

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.type import TypeAcceleration, TypeMonotone
from commonroad_crime.measure.distance.hw import HW
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class DST(CriMeBase):
    """
    https://criticality-metrics.readthedocs.io/en/latest/time-scale/THW.html
    the deceleration that has to be applied to a vehicle to maintain a certain safety time
    """
    measure_name = TypeAcceleration.DST
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(DST, self).__init__(config)
        self._hw_solver = HW(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step
        # under the assumption that the velocity of the other object remains constant
        headway = self._hw_solver.compute(vehicle_id, time_step)
        if headway < 0:
            return -math.inf
        v_ego = self.ego_vehicle.state_at_time(time_step).velocity
        v_otr = self.other_vehicle.state_at_time(time_step).velocity
        # from (17) in Schubert, Robin, Karsten Schulze, and Gerd Wanielik. "Situation assessment for automatic
        # lane-change maneuvers." IEEE Transactions on Intelligent Transportation Systems 11.3 (2010): 607-616.
        dst = 3 * (v_ego - v_otr)**2 / (2 * (headway - v_otr * self.configuration.acceleration.safety_time))
        self.value = utils_gen.int_round(dst, 2)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self):
        self._hw_solver.configuration.debug.draw_visualization = False
        self._hw_solver.visualize()
        plt.title(f"{self.measure_name} of {self.value} m")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()
