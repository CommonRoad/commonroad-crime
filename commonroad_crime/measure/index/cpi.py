__author__ = "Yuanfei Lin, Sicheng Wang"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import logging
import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as plt
import matplotlib.cm as cm

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
        # We assume the MADR (aka. Maximum Available Deceleration Rate)
        # is normally distributed.
        self.cpi_config = self.configuration.index.cpi
        self._madr_dist = norm(self.cpi_config.madr_mean,
                               self.cpi_config.madr_devi)
        #  Survival function: gives the probability of obtaining a value larger than the given value
        self.tr_ub_prob = self._madr_dist.sf(self.cpi_config.madr_uppb)
        self.tr_lb_prob = self._madr_dist.cdf(self.cpi_config.madr_lowb)
        self.a_lon_req_list = []
        self.value = 0

    def compute(self,
                vehicle_id: int,
                time_step: int = 0,
                verbose: bool = True):
        utils_log.print_and_log_info(
            logger, f"* Computing the {self.measure_name} between ego vehicle"
            f" and vehicle {vehicle_id} at timestep {time_step}.")

        self.time_step = time_step
        self.end_time_step = self.ego_vehicle.prediction.final_time_step
        self.value = 0

        for ts in range(self.time_step, self.end_time_step):
            try:
                a_lon_req = self._a_lon_req_object.compute(vehicle_id, ts)
                self.a_lon_req_list.append(a_lon_req)
            except ValueError:
                utils_log.print_and_log_info(
                    logger,
                    f"*\t\t vehicle {vehicle_id} is no longer in the lanelets.",
                    verbose)
                self.a_lon_req_list.append(self.cpi_config.madr_uppb)
                continue
            if a_lon_req >= self.cpi_config.madr_uppb:
                continue
            elif a_lon_req <= self.cpi_config.madr_lowb:
                self.value += 1
            else:
                # P(ALonReq>MADR) is equal to integral of PDF
                # from AlonReq to upperbound.
                # -inf___lowerbound___ALonReq___upperbound___0___+inf
                self.value += (self._madr_dist.sf(a_lon_req) - self.tr_ub_prob
                              ) / (1 - self.tr_lb_prob - self.tr_ub_prob)

        # Normalize the result with timespan.
        try:
            self.value /= (self.end_time_step - self.time_step)
        except ZeroDivisionError:
            utils_log.print_and_log_error(logger, f"Timespan is zero.")

        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}.")
        return self.value

    def visualize(self):
        if len(self.a_lon_req_list) == 0:
            utils_log.print_and_log_error(logger, f"No valid Data to plot.")
            return

        fig, ax = plt.subplots()
        x = np.linspace(self.cpi_config.madr_lowb, self.cpi_config.madr_uppb,
                        100)
        # Plot the MADR distribution first.
        ax.plot(x, self._madr_dist.pdf(x), label='MADR')
        ax.legend()
        # Build colormap
        cmap = cm.get_cmap("viridis", len(self.a_lon_req_list))
        colors = cmap(np.linspace(0, 1, len(self.a_lon_req_list)))
        # Plot datapoint at different time with different color
        for dr, c in zip(self.a_lon_req_list, colors):
            scatter = ax.scatter(dr, self._madr_dist.pdf(dr), color=c)
        scatter.set_clim(self.time_step, self.end_time_step)
        # Draw colorbar
        cbar = plt.colorbar(scatter, ax=ax, orientation='horizontal')
        cbar.set_label('Time Step')

        plt.title("Deceleration Rate avoiding collision")
        plt.xlabel('Deceleration')

        plt.show()
