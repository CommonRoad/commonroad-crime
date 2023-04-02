__author__ = "Oliver Specht, Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import matplotlib.pyplot as plt
import numpy as np
import logging

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
from commonroad_crime.measure.time.ttc import TTC
import commonroad_crime.utility.visualization as utils_vis
from commonroad_crime.utility.visualization import TUMcolor
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.general as utils_gen

logger = logging.getLogger(__name__)


class TIT(CriMeBase):
    """
    Minderhoud, Michiel, M and Bovy, Piet, H.L.,
    “Extended time-to-collision measures for road traffic
    safety assessment,” Accident Analysis & Prevention,
    vol. 33, pp. 89–97, 2001.
    """
    measure_name = TypeTime.TIT
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(TIT, self).__init__(config)
        self.ttc_object = TTC(config)
        self._ttc_cache = dict()

    def compute(self, vehicle_id: int, time_step: int = 0):
        """
        Iterate through all states, calculate ttc, compare it to tau and then add dt to the result
        if ttc is smaller than tau
        """
        # init
        tau = self.configuration.time.tau
        state_list = self.ego_vehicle.prediction.trajectory.state_list
        self._ttc_cache.clear()

        self.value = 0
        for i in range(time_step, len(state_list)):
            ttc_result = self.ttc_object.compute(vehicle_id, i)
            self._ttc_cache[i] = ttc_result
            if ttc_result <= tau:
                self.value += (tau - ttc_result) * self.dt
        self.value = utils_gen.int_round(self.value, 4)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def visualize(self):
        plt.cla()
        tau = self.configuration.time.tau

        plt.xlabel('time step')
        plt.ylabel('TTC')

        # Extract the time_step and ttc_result from the cache dictionary
        time_step_list = list(self._ttc_cache.keys())
        ttc_result_list = [val for val in self._ttc_cache.values()]

        # Find the indices where ttc_result_list < tau
        below_tau_indices = np.where(np.array(ttc_result_list) < tau)[0]

        # Plot the time_step vs. ttc_result curve
        plt.plot(time_step_list, ttc_result_list, label='TTC curve')

        # Plot the tau curve as a horizontal line
        plt.axhline(y=tau, color=TUMcolor.TUMred, linestyle='--', label='Tau')

        # Fill the region where ttc_result < tau with the predefined color
        for i in range(1, len(below_tau_indices)):
            idx1 = below_tau_indices[i - 1]
            idx2 = below_tau_indices[i]
            plt.fill_between(time_step_list[idx1:idx2 + 1], ttc_result_list[idx1:idx2 + 1],
                             tau, color=TUMcolor.TUMred, alpha=0.5)

        plt.legend()

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
            else:
                plt.show()

