__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging
import matplotlib.pyplot as plt

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeIndexScale, TypeMonotone
from commonroad_crime.measure.acceleration_scale.a_long_req import ALongReq
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis

logger = logging.getLogger(__name__)


class BTN(CriMeBase):
    """
    the relation between the negative acceleration needed to marginally avoid a collision and the maximum deceleration
    available for the vehicle.

    -- from Åsljung, Daniel, Jonas Nilsson, and Jonas Fredriksson. "Using extreme value theory for vehicle level safety
    validation and implications for autonomous vehicles." IEEE Transactions on Intelligent Vehicles 2.4 (2017): 288-297.
    """
    measure_name = TypeIndexScale.BTN
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(BTN, self).__init__(config)
        self._a_long_req_object = ALongReq(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self.set_other_vehicles(vehicle_id)
        self.time_step = time_step
        a_long_req = self._a_long_req_object.compute(vehicle_id, time_step)
        # (9) in "Using extreme value theory for vehicle level safety validation and implications
        # for autonomous vehicles."
        self.value = utils_gen.int_round(a_long_req / self.configuration.vehicle.curvilinear.a_lon_min, 4)
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
