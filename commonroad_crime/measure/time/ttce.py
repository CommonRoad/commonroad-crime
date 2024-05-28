__author__ = "Oliver Specht, Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.4.1"
__maintainer__ = "beta"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import matplotlib.pyplot as plt
import numpy as np
import logging
import math

from commonroad_crime.measure.distance.dce import DCE
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.visualization as utils_vis


logger = logging.getLogger(__name__)


class TTCE(CriMeBase):
    """
    The TTCE is a distance-dependent risk indicator, which generalizes the concept of the TTC to the non-collision
    case. J. Eggert, “Predictive risk estimation for intelligent ADAS functions,” in 17th International Conference on
    Intelligent Transportation Systems (ITSC), pp. 711–718, IEEE, 2014.
    """

    measure_name = TypeTime.TTCE
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(TTCE, self).__init__(config)
        self._dce_object = DCE(config)

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        """
        Using DCE to calculate the TTCE value. DCE marks the time step when the minimal distance is reached.
        """
        if not self.validate_update_states_log(vehicle_id, time_step, verbose):
            return np.nan

        self._dce_object.compute(vehicle_id, self.time_step, verbose=verbose)
        if self._dce_object.time_dce is not math.inf:
            self.value = utils_gen.int_round(
                (self._dce_object.time_dce - self.time_step) * self.dt, 3
            )
        else:
            self.value = self._dce_object.time_dce
        utils_log.print_and_log_info(
            logger,
            f"*\t\t {self.measure_name} with vehicle id {vehicle_id} = {self.value}",
            verbose,
        )
        return self.value

    def visualize(self):
        self._dce_object.configuration.debug.draw_visualization = False
        self._dce_object.visualize()
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(
                self.measure_name,
                self.configuration.general.path_output,
                self.time_step,
            )
        else:
            plt.show()
