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
from commonroad_crime.data_structure.type import TypeIndexScale
from commonroad_crime.metric.distance_scale.hw import HW
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.solver as utils_sol
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class BTN(CriMeBase):
    """
    the relation between the negative acceleration needed to marginally avoid a collision and the maximum deceleration
    available for the vehicle.

    -- from Åsljung, Daniel, Jonas Nilsson, and Jonas Fredriksson. "Using extreme value theory for vehicle level safety
    validation and implications for autonomous vehicles." IEEE Transactions on Intelligent Vehicles 2.4 (2017): 288-297.
    """
    metric_name = TypeIndexScale.BTN

    def __init__(self, config: CriMeConfiguration):
        super(BTN, self).__init__(config)
        self._hw_object = HW(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        self._set_other_vehicles(vehicle_id)
        self.time_step = time_step
        if not utils_gen.check_in_same_lanelet(self.sce.lanelet_network, self.ego_vehicle,
                                               self.other_vehicle, time_step):
            utils_log.print_and_log_info(logger, f"*\t\t vehicle {vehicle_id} is not in the same lanelet as the "
                                                 f"ego vehicle {self.ego_vehicle.obstacle_id}")
            self.value = 0.  # no negative acceleration is needed for avoiding a collision
            utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
            return self.value
        if hasattr(self.other_vehicle.state_at_time(time_step), 'acceleration'):
            a_obj = self.other_vehicle.state_at_time(time_step).acceleration
        elif self.other_vehicle.state_at_time(time_step + 1):
            a_obj = utils_sol.compute_acceleration(self.other_vehicle.state_at_time(time_step).velocity,
                                                   self.other_vehicle.state_at_time(time_step).velocity,
                                                   self.dt)
        else:
            a_obj = 0.
        # compute the relative longitudinal velocity
        v_rel = self.ego_vehicle.state_at_time(time_step).velocity-self.other_vehicle.state_at_time(time_step).velocity
        # compute the headway distance
        x_rel = self._hw_object.compute(vehicle_id, time_step)
        a_req = a_obj - v_rel**2/(2*x_rel)
        if a_req > 0:  # the object is non-closing
            self.value = 0.
        else:
            # (9) in "Using extreme value theory for vehicle level safety validation and implications
            # for autonomous vehicles."
            self.value = utils_gen.int_round(a_req / self.configuration.vehicle.curvilinear.a_lon_min, 2)
        utils_log.print_and_log_info(logger, f"*\t\t {self.metric_name} = {self.value}")
        return self.value

    def visualize(self):
        self._initialize_vis(plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                              self.ego_vehicle.prediction.
                                                                              trajectory.state_list,
                                                                              margin=10))
        self.other_vehicle.draw(self.rnd, {'time_begin': self.time_step, **utils_vis.OTHER_VEHICLE_DRAW_PARAMS})
        self.rnd.render()
        plt.title(f"{self.metric_name} at time step {self.time_step}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()

