__author__ = "Yuanfei Lin, Ziqian Xu"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.3.0"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "beta"

import math
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeDistance, TypeMonotone
import commonroad_crime.utility.logger as utils_log
import commonroad_crime.utility.visualization as utils_vis
import matplotlib.pyplot as plt
import logging
import numpy as np
import commonroad_crime.utility.general as utils_gen
from commonroad_crime.utility.visualization import TUMcolor
from commonroad_crime.measure.distance.msd import MSD
from commonroad_crime.measure.time.et import ET
from commonroad.geometry.polyline_util import (
    compute_total_polyline_length,
)


logger = logging.getLogger(__name__)


class PSD(CriMeBase):
    """
    The definition is obtained from: B. L. Allen, B. T. Shin, and P. J. Cooper, “Analysis of Traffic Conflicts and
    Collisions,” Transportation Research Record, vol. 667, pp. 67–74, 1978.
    """

    measure_name = TypeDistance.PSD
    monotone = TypeMonotone.NEG

    def __init__(self, config: CriMeConfiguration):
        super(PSD, self).__init__(config)
        self._msd_object = MSD(config)
        self._et_object = ET(config)

    def compute(self, vehicle_id: int = None, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(
            logger,
            f"* Computing the {self.measure_name} beginning at time step {time_step}",
            verbose,
        )
        self.time_step = time_step
        self.set_other_vehicles(vehicle_id)
        # compute MSD
        if isinstance(self.other_vehicle, DynamicObstacle):
            self._et_object.ca = self._et_object.get_ca(
                self.time_step, self.other_vehicle
            )
            if self._et_object.ca is not None:
                # check when the ego vehicle enters the conflict area
                _, enter_time, _ = self._et_object.get_ca_time_info(
                    self.ego_vehicle, self.time_step, self._et_object.ca
                )
                # compute the remaining distance using polyline
                if enter_time is not math.inf:
                    state_list = self.ego_vehicle.prediction.trajectory.state_list[
                        self.time_step : enter_time + 1
                    ]
                    pos_list = np.asarray([state.position for state in state_list])
                    if pos_list.shape[0] < 2:
                        # already within the conflict area
                        rd = 0.0
                    else:
                        rd = compute_total_polyline_length(pos_list)
                    utils_log.print_and_log_info(
                        logger,
                        f"*\t\t The remaining distance to the potential point of collision is {rd:0.2f}",
                        verbose,
                    )
                    self.value = utils_gen.int_round(
                        rd
                        / self._msd_object.compute(
                            vehicle_id, time_step, verbose=verbose
                        ),
                        2,
                    )
                    utils_log.print_and_log_info(
                        logger, f"*\t\t {self.measure_name} = {self.value}", verbose
                    )
                    return self.value
                else:
                    utils_log.print_and_log_info(
                        logger,
                        f"*\t\t The ego vehicle will not enter the conflict area",
                        verbose,
                    )
                    return math.inf
            else:
                utils_log.print_and_log_info(
                    logger,
                    f"*\t\t No valid conflict area exists in this scenario",
                    verbose,
                )
                return math.inf
        else:
            utils_log.print_and_log_info(
                logger,
                f"*\t\t Vehicle {self.other_vehicle} is not a dynamic obstacle, "
                f"thus the conflict area does not exist",
                verbose,
            )
            return math.inf

    def visualize(self, figsize: tuple = (25, 15)):
        if self.configuration.debug.plot_limits:
            plot_limits = self.configuration.debug.plot_limits
        else:
            plot_limits = utils_vis.plot_limits_from_state_list(
                self.time_step,
                self.ego_vehicle.prediction.trajectory.state_list,
                margin=10,
            )

        if self._et_object.ca is not None and self.value is not math.inf:
            self._initialize_vis(figsize=figsize, plot_limit=plot_limits)
            self.rnd.render()

            utils_vis.draw_state_list(
                self.rnd,
                self.ego_vehicle.prediction.trajectory.state_list[self.time_step :],
                color=TUMcolor.TUMblue,
                linewidth=1,
            )
            utils_vis.draw_state_list(
                self.rnd,
                self.other_vehicle.prediction.trajectory.state_list[self.time_step :],
                color=TUMcolor.TUMlightgray,
                linewidth=1,
            )
            utils_vis.draw_dyn_vehicle_shape(
                self.rnd,
                self.ego_vehicle,
                time_step=self.time_step,
                color=TUMcolor.TUMgreen,
            )
            msd_location = self._msd_object.compute_msd_location_time_step(
                self._msd_object.value
            )
            utils_vis.draw_circle(self.rnd, msd_location, 0.5, 1, color=TUMcolor.TUMred)

            _, enter_time, _ = self._et_object.get_ca_time_info(
                self.ego_vehicle, self.time_step, self._et_object.ca
            )
            utils_vis.draw_dyn_vehicle_shape(
                self.rnd,
                self.ego_vehicle,
                time_step=enter_time,
                color=TUMcolor.TUMgreen,
            )
            utils_vis.draw_dyn_vehicle_shape(
                self.rnd,
                self.other_vehicle,
                time_step=self.time_step,
                color=TUMcolor.TUMdarkred,
            )
            x_i, y_i = self._et_object.ca.exterior.xy
            plt.plot(x_i, y_i, color=TUMcolor.TUMblack, zorder=1001)
            plt.fill(x_i, y_i, color=TUMcolor.TUMred, zorder=1001)

            plt.title(f"{self.measure_name} of {self.time_step} time steps")

            if self.configuration.debug.draw_visualization:
                if self.configuration.debug.save_plots:
                    utils_vis.save_fig(
                        self.measure_name,
                        self.configuration.general.path_output,
                        self.time_step,
                    )
                else:
                    plt.show()
