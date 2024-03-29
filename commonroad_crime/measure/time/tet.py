__author__ = "Oliver Specht, Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.4.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import logging
import numpy as np

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeTime, TypeMonotone
from commonroad_crime.measure.time.tit import TIT
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.general as utils_gen

logger = logging.getLogger(__name__)


class TET(TIT):
    """
    Minderhoud, Michiel, M and Bovy, Piet, H.L., “Extended time-to-collision measures for road traffic safety
    assessment,” Accident Analysis & Prevention, vol. 33, pp. 89–97, 2001.
    """

    measure_name = TypeTime.TET
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(TET, self).__init__(config)

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        """
        Iterate through all states, calculate ttc, compare it to tau and then add dt to the result
        if ttc is smaller than tau
        """
        if not self.validate_update_states_log(vehicle_id, time_step, verbose):
            return np.nan
        tau = self.configuration.time.tau
        state_list = self.ego_vehicle.prediction.trajectory.state_list

        self.value = 0
        for i in range(time_step, len(state_list)):
            ttc_result = self.ttc_object.compute(vehicle_id, i, verbose=verbose)
            self._ttc_cache[i] = ttc_result
            if ttc_result <= tau:
                self.value += self.dt
        self.value = utils_gen.int_round(self.value, 4)
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}", verbose
        )
        return self.value
