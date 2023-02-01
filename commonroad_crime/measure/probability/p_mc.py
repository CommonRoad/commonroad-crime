__author__ = "Yuanfei Lin"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["KoSi"]
__version__ = "0.0.1"
__maintainer__ = "Yuanfei Lin"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Pre-alpha"

import numpy as np
import matplotlib.pyplot as plt
import logging

from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle

from commonroad_crime.data_structure.base import CriMeBase
from commonroad_crime.utility.simulation import (SimulationLongMonteCarlo, SimulationLatMonteCarlo, Maneuver,
                                                 SimulationRandoMonteCarlo)
from commonroad_crime.measure.time.ttc_star import TTCStar
from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.data_structure.type import TypeProbability, TypeMonotone
import commonroad_crime.utility.visualization as utils_vis
import commonroad_crime.utility.general as utils_gen
import commonroad_crime.utility.logger as utils_log
from commonroad_crime.utility.visualization import TUMcolor

logger = logging.getLogger(__name__)


class P_MC(CriMeBase):
    """
    P-MC produces a collision probability estimation based on future evolutions from a Monte Carlo path planning
    prediction.

    See Broadhurst, Adrian, Simon Baker, and Takeo Kanade. "Monte Carlo road safety reasoning." IEEE Proceedings of
    Intelligent Vehicles Symposium IEEE, 2005.
    """
    measure_name = TypeProbability.P_MC
    monotone = TypeMonotone.POS

    def __init__(self,
                 config: CriMeConfiguration):
        super(P_MC, self).__init__(config)

        self.maneuver_list = [Maneuver.STOPMC, Maneuver.TURNMC, Maneuver.LANECHANGEMC,
                              Maneuver.OVERTAKEMC, Maneuver.RANDOMMC]
        config_mc = self.configuration.probability.monte_carlo
        assert len(self.maneuver_list) == len(
            config_mc.mvr_weights), "Please follow the configuration guide for defining the weights of the maneuvers!"
        if config_mc.nr_samples < len(self.maneuver_list):
            id_random_mvr = np.random.choice(range(len(self.maneuver_list)), size=config_mc.nr_samples)
            self.maneuver_list = [self.maneuver_list[id_m] for id_m in id_random_mvr]
            config_mc.mvr_weights = np.array(config_mc.mvr_weights)[id_random_mvr]
        self.nr_samples = config_mc.nr_samples
        self.sample_prob = np.array(config_mc.mvr_weights) / np.sum(config_mc.mvr_weights)
        self.ego_state_list_set_cf = []  # collision-free
        self.ego_state_list_set_wc = []  # with collisions
        self.sim_time_steps = int(config_mc.prediction_horizon / self.sce.dt)
        self.ttc_object = TTCStar(self.configuration)

    def compute(self, time_step: int = 0, vehicle_id = None, verbose: bool = True):
        utils_log.print_and_log_info(logger, f"* Computing the {self.measure_name} at time step {time_step}", verbose)
        utils_log.print_and_log_info(logger, f"* \t\t nr of samples "
                                             f"{self.configuration.probability.monte_carlo.nr_samples}")
        self.time_step = time_step
        colliding_prob_list = []
        for i in range(len(self.maneuver_list)):
            maneuver = self.maneuver_list[i]
            # randomly rounding to integer
            nr_sample_maneuver = int(self.nr_samples * self.sample_prob[i] + np.random.random())
            ego_sl_bundle, pdf_bundle = self.monte_carlo_simulation(self.ego_vehicle, maneuver, nr_sample_maneuver)
            for j in range(len(ego_sl_bundle)):
                if self.ttc_object.detect_collision(ego_sl_bundle[j]):
                    colliding_prob_list.append(pdf_bundle[j])
                    self.ego_state_list_set_wc.append(ego_sl_bundle[j])
                else:
                    self.ego_state_list_set_cf.append(ego_sl_bundle[j])
        # (14) in Broadhurst, Adrian, Simon Baker, and Takeo Kanade. "Monte Carlo road safety reasoning." IEEE
        # Proceedings of Intelligent Vehicles Symposium, IEEE, 2005.
        p_mc = np.average(np.array(colliding_prob_list))
        self.value = utils_gen.int_round(p_mc, 4)
        utils_log.print_and_log_info(logger, f"*\t\t {self.measure_name} = {self.value}")
        return self.value

    def monte_carlo_simulation(self, vehicle: DynamicObstacle, maneuver: Maneuver, nr_samples: int):
        """
        Monte Carlo simulation of the given vehicle.
        :param vehicle: dynamic obstacles
        :param maneuver: maneuver of vehicle
        :param nr_samples: number of samples
        """
        # static obstacle: no trajectory is simulated
        if isinstance(vehicle, StaticObstacle):
            msg = "There are no trajectories that can be simulated for static obstacles"
            utils_log.print_and_log_error(logger, msg)
            raise ValueError(msg)
        state_list_bundle = []
        pdf_bundle = []
        if maneuver in [Maneuver.STOPMC]:
            simulator = SimulationLongMonteCarlo(maneuver, vehicle, self.configuration)
        elif maneuver in [Maneuver.TURNMC, Maneuver.OVERTAKEMC, Maneuver.LANECHANGEMC]:
            # change the lane width mode
            self.configuration.time.steer_width = 2
            simulator = SimulationLatMonteCarlo(maneuver, vehicle, self.configuration)
        elif maneuver in [Maneuver.RANDOMMC]:
            simulator = SimulationRandoMonteCarlo(maneuver, vehicle, self.configuration)
        else:
            return state_list_bundle
        for _ in range(nr_samples):
            state_list_bundle.append(simulator.simulate_state_list(self.time_step, self.sim_time_steps))
            pdf_bundle.append(simulator.pdf)
        return state_list_bundle, pdf_bundle

    def visualize(self, figsize: tuple = (25, 15)):
        self._initialize_vis(figsize=figsize,
                             plot_limit=utils_vis.plot_limits_from_state_list(self.time_step,
                                                                              self.ego_vehicle.prediction.
                                                                              trajectory.state_list,
                                                                              margin=20))
        self.rnd.render()
        for sl in self.ego_state_list_set_wc:
            utils_vis.draw_state_list(self.rnd, sl, color=TUMcolor.TUMred)
        for sl in self.ego_state_list_set_cf:
            utils_vis.draw_state_list(self.rnd, sl, color=TUMcolor.TUMblue)
        plt.title(f"{self.measure_name} at time step {self.time_step} is {self.value}")
        if self.configuration.debug.save_plots:
            utils_vis.save_fig(self.measure_name, self.configuration.general.path_output, self.time_step)
        else:
            plt.show()
