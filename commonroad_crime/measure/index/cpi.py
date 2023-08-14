__author__ = "Yuanfei Lin, Sicheng Wang"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import logging
import math

import numpy as np
from scipy.stats import truncnorm
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeIndex, TypeMonotone
from commonroad_crime.measure.acceleration.a_long_req import ALongReq
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.logger as utils_log

logger = logging.getLogger(__name__)


class CPI(CriMeBase):
    """
    The CPI is a scenario-level metric and calculates the average probability
    that a vehicle can not avoid a collision by deceleration.

    -- from Cunto, Flavio Jose Craveiro, and Frank F. Saccomanno. Microlevel traffic simulation method for assessing
    crash potential at intersections. No. 07-2180. 2007.
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
        a = (
            self.cpi_config.madr_lowb - self.cpi_config.madr_mean
        ) / self.cpi_config.madr_devi
        b = (
            self.cpi_config.madr_uppb - self.cpi_config.madr_mean
        ) / self.cpi_config.madr_devi
        self._madr_dist = truncnorm(
            a, b, self.cpi_config.madr_mean, self.cpi_config.madr_devi
        )
        self.dr_lon_req_list = []
        self.value = 0
        self.end_time_step = self.ego_vehicle.prediction.final_time_step

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(
            logger,
            f"* Computing the {self.measure_name} between ego vehicle"
            f" and vehicle {vehicle_id} at timestep {time_step}.",
            verbose,
        )

        self.time_step = time_step
        self.end_time_step = self.ego_vehicle.prediction.final_time_step
        self.value = 0.0

        for ts in range(self.time_step, self.end_time_step):
            try:
                dr_lon_req = -self._a_lon_req_object.compute(vehicle_id, ts)
                self.dr_lon_req_list.append(dr_lon_req)
            except ValueError:
                utils_log.print_and_log_info(
                    logger,
                    f"*\t\t vehicle {vehicle_id} is no longer in the lanelets.",
                    verbose,
                )
                self.dr_lon_req_list.append(self.cpi_config.madr_lowb)
                continue
            if dr_lon_req <= self.cpi_config.madr_lowb:
                continue
            else:
                # P(ALonReq>MADR)
                # dr_long_req_norm = (dr_lon_req - self.cpi_config.madr_mean) / self.cpi_config.madr_devi
                if not math.isnan(self._madr_dist.cdf(dr_lon_req)):
                    self.value += self._madr_dist.cdf(dr_lon_req)

        # Normalize the result with timespan.
        try:
            self.value /= self.end_time_step - self.time_step
        except ZeroDivisionError:
            utils_log.print_and_log_error(logger, f"*\t\t Timespan is zero.", verbose)

        utils_log.print_and_log_info(
            logger, f"*\t\t {self.measure_name} = {self.value}.", verbose
        )
        return float(self.value)

    def visualize(self):
        num_data = len(self.dr_lon_req_list)
        if num_data == 0:
            utils_log.print_and_log_error(logger, f"No valid Data to plot.")
            return

        fig, ax = plt.subplots()
        x = np.linspace(self.cpi_config.madr_lowb, self.cpi_config.madr_uppb, 100)
        # Plot the MADR distribution first.
        ax.plot(x, self._madr_dist.pdf(x), label="MADR")
        ax.legend()
        # Build colormap
        cmap = cm.get_cmap("viridis", num_data)
        colors = cmap(np.linspace(0, 1, num_data))
        # Plot datapoint at different time with different color

        scatter = None
        for dr, c in zip(self.dr_lon_req_list, colors):
            scatter = ax.scatter(dr, self._madr_dist.pdf(dr), color=c)
        if scatter is not None:
            scatter.set_clim(self.time_step, self.end_time_step)
            # Draw color bar
            cbar = plt.colorbar(scatter, ax=ax, orientation="horizontal")
            cbar.set_label("Time Step")

        plt.title(f"{self.measure_name} of {self.value}")
        plt.xlabel("Deceleration")

        if self.configuration.debug.draw_visualization:
            if self.configuration.debug.save_plots:
                utils_vis.save_fig(
                    self.measure_name,
                    self.configuration.general.path_output,
                    self.time_step,
                )
            else:
                plt.show()
