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

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTimeScale
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis

logger = logging.getLogger(__name__)


class TTC(CriMeBase):
    """
    With a constant acceleration decision model of the vehicles motion

    -- using (5.24) in "Collision Avoidance Theory with Application to Automotive Collision Mitigation"
    """
    metric_name = TypeTimeScale.TTC

    def __init__(self, config: CriMeConfiguration):
        super(TTC, self).__init__(config)

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}")
        self._set_other_vehicles(vehicle_id)
        self.time_step = time_step
        ######### to be implemented ##############
        self.value = 0.5 #math.inf
        ##########################################
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

