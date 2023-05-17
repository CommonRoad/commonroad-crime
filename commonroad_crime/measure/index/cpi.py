__author__ = "Yuanfei Lin, Sicheng Wang"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging
from scipy.stats import norm

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeIndex, TypeMonotone
from commonroad_crime.measure.acceleration.a_long_req import ALongReq
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class CPI(CriMeBase):
    """
    The CPI is a scenario-level metric and calculates the average probability
    that a vehicle can not avoid a collision by deceleration.

    - from Cunto, Flavio Jose Craveiro, and Frank F. Saccomanno.
    Microlevel traffic simulation method for assessing crash potential at
    intersections. No. 07-2180. 2007.
    """

    measure_name = TypeIndex.CPI
    monotone = TypeMonotone.POS

    def __init__(self, config: CriMeConfiguration):
        super(CPI, self).__init__(config)
        self._a_lon_req_object = ALongReq(config)
        # TODO Add interface for different a_lon_min.
        # self._a_lon_min_mean = self.configuration.vehicle.curvilinear.a_lon_min
        # We assume the MADR(aka. Maximum Avaliable Deceleration Rate) is normally distributed.
        self.cpi_config = self.configuration.index.cpi
        self._madr_dist = norm(self.cpi_config.madr_mean,
                               self.cpi_config.madr_devi)
        self.tr_ub_prob = self._madr_dist.sf(self.cpi_config.madr_uppb)
        self.tr_lb_prob = self._madr_dist.cdf(self.cpi_config.madr_lowb)
        self.value = 0

    def compute(self, vehicle_id: int, time_step: int = 0):
        utils_log.print_and_log_info(
            logger,
            f"* Computing the {self.measure_name} of ego vehicle in given scenario."
        )

        self.start_time_step = time_step
        a_lon_req = 0
        self.final_time_step = self.ego_vehicle.prediction.final_time_step

        for ts in range(self.start_time_step, self.final_time_step):
            try:
                a_lon_req = self._a_lon_req_object.compute(vehicle_id, ts)
            except ValueError:
                print("Vehicle %d is no longer in the lanelets." % vehicle_id)
                continue
            if a_lon_req >= self.cpi_config.madr_uppb:
                continue
            elif a_lon_req <= self.cpi_config.madr_lowb:
                self.value += 1
            else:
                # P(ALonReq>MADR) is equal to intergral of PDF from AlonReq to upperbound.
                # -inf___lowerbound___ALonReq___upperbound___0___+inf
                self.value += (self._madr_dist.sf(a_lon_req) - self.tr_ub_prob
                              ) / (1 - self.tr_lb_prob - self.tr_ub_prob)

                # Nomalize the result with timespan.
        self.value /= (self.final_time_step - self.start_time_step)
        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}.")
        return self.value

    def visualize(self):
        pass
