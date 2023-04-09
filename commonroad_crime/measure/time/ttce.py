__author__ = "Oliver Specht, Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import matplotlib.pyplot as plt
import logging

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
    See https://criticality-metrics.readthedocs.io/
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
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}")
        self._dce_object.compute(vehicle_id, time_step)
        self.value = utils_gen.int_round((self._dce_object.time_dce - time_step) * self.dt, 3)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} with vehicle id {vehicle_id} = {self.value}")
        return self.value

    def visualize(self):
        self._dce_object.configuration.debug.draw_visualization = False
        self._dce_object.visualize()
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()

