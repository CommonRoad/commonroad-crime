__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Optional
import logging

from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.utility.simulation import SimulationLongMonteCarlo, SimulationLatMonteCarlo, Maneuver
from commonroad_crime.metric.time_scale.ttc import TTC
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeProbabilityScale
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class P_MC(CriMeBase):
    """
    P-MC produces a collision probability estimation based on future evolutions from a Monte Carlo path planning
    prediction.

    See Broadhurst, Adrian, Simon Baker, and Takeo Kanade. "Monte Carlo road safety reasoning." IEEE Proceedings.
    Intelligent Vehicles Symposium, 2005.. IEEE, 2005.
    """
    metric_name = TypeProbabilityScale.P_MC

    def __init__(self,
                 config: CriMeConfiguration):
        super(P_MC, self).__init__(config)

        self.maneuver_list = [Maneuver.STOPMC, Maneuver.TURNMC, Maneuver.LANECHANGEMC,
                              Maneuver.OVERTAKEMC, Maneuver.RANDOMMC]
        config_mc = self.configuration.probability_scale.monte_carlo
        assert len(self.maneuver_list) == len(
            config_mc.mvr_weights), "Please follow the configuration guide for defining the weights of the maneuvers!"
        self.sample_nr_list = config_mc.nr_samples * np.asarray(config_mc.mvr_weights) / np.sum(config_mc.mvr_weights)
        self.ego_state_list_set = []
        self.other_state_list_set = []
        self.sim_time_steps = int(config_mc.prediction_horizon/self.sce.dt)

    def compute(self, vehicle_id: int, time_step: int = 0, verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Computing the {self.metric_name} at time step {time_step}", verbose)
        self._set_other_vehicles(vehicle_id)
        self.time_step = time_step
        self.ego_state_list_set = self.monte_carlo_simulation(self.ego_vehicle)
        self.other_state_list_set = self.monte_carlo_simulation(self.other_vehicle)
        colliding_sample_nr = 0
        for i in range(len(self.other_state_list_set)):
            for j in range(len(self.ego_state_list_set)):
                if isinstance(self.other_vehicle, StaticObstacle):
                    ttc_object = TTC(self.configuration)
                else:
                    # to avoid the error:  AssertionError: state_list[0].time_step=0 != self.initial_time_step=1
                    self.other_vehicle.prediction.trajectory.state_list = self.other_state_list_set[i][
                        self.other_vehicle.prediction.initial_time_step:
                    ]
                    ttc_object = TTC(self.configuration)
                if ttc_object.detect_collision(self.ego_state_list_set[j]):
                    colliding_sample_nr += 1
                    print(colliding_sample_nr)

    def monte_carlo_simulation(self, vehicle: DynamicObstacle):
        """
        Monte Carlo simulation of the given vehicle.
        :param vehicle: dynamic obstacles
        """
        # static obstacle: no trajectory is simulated
        if isinstance(vehicle, StaticObstacle):
            msg = "There are no trajectories that can be simulated for static obstacles"
            utils_log.print_and_log_error(logger, msg)
            raise ValueError(msg)
        state_list_bundle = []
        for i in range(len(self.maneuver_list)):
            mvr = self.maneuver_list[i]
            if mvr in [Maneuver.STOPMC]:
                simulator = SimulationLongMonteCarlo(mvr, vehicle, self.configuration)
            elif mvr in [Maneuver.TURNMC, Maneuver.OVERTAKEMC, Maneuver.LANECHANGEMC]:
                simulator = SimulationLatMonteCarlo(mvr, vehicle, self.configuration)
            else:
                return state_list_bundle
            for _ in range(int(self.sample_nr_list[i])):
                state_list_bundle.append(simulator.simulate_state_list(self.time_step, self.sim_time_steps))
        return state_list_bundle

    def visualize(self, figsize: tuple = (25, 15)):
        self._initialize_vis(figsize=figsize, plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                                               self.ego_state_list_set[-1],
                                                                                               margin=20))
        self.rnd.render()

        for sl in self.ego_state_list_set:
            utils_vis.draw_state_list(self.rnd, sl, color=TUMcolor.TUMblue)
        for sl in self.other_state_list_set:
            utils_vis.draw_state_list(self.rnd, sl, color=TUMcolor.TUMred)

        #plt.title(f"{self.metric_name} at time step {tstm - self.time_step}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.metric_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()


