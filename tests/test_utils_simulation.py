"""
Unit tests of the utility functions for simulation
"""

import unittest

from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_crime.utility.logger as util_logger
from commonroad_crime.utility.simulation import (SimulationLong, SimulationLat, Maneuver,
                                                 SimulationLongMonteCarlo, SimulationLatMonteCarlo,
                                                 SimulationRandoMonteCarlo)
import commonroad_crime.utility.visualization as utils_vis


class TestSimulation(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Urban-3_3_Repair'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()
        self.ego_vehicle = self.config.scenario.obstacle_by_id(self.config.vehicle.ego_id)

        self.rnd = MPRenderer(plot_limits=[50, 100, -10, 12.5])
        self.config.scenario.draw(self.rnd)
        self.rnd.render()

    def test_simulation_long(self):
        sim_long = SimulationLong(Maneuver.BRAKE, self.ego_vehicle, self.config)
        simulated_state1 = sim_long.simulate_state_list(0)
        for i in range(len(simulated_state1)):
            self.assertEqual(simulated_state1[i].time_step, i)
        self.assertEqual(
            simulated_state1[-1].time_step,
            self.ego_vehicle.prediction.final_time_step)

        sim_long.update_maneuver(Maneuver.KICKDOWN)
        simulated_state2 = sim_long.simulate_state_list(20)

        sim_long.update_maneuver(Maneuver.CONSTANT)
        simulated_state3 = sim_long.simulate_state_list(10)

        for sim_state_list in [simulated_state1, simulated_state2, simulated_state3]:
            utils_vis.draw_state_list(self.rnd, sim_state_list, 0)

        utils_vis.save_fig("test_simulate_long", self.config.general.path_output, 0)

    def test_simulation_long_mc(self):
        sim_stat_list_total = []
        sim_long = SimulationLongMonteCarlo(Maneuver.STOPMC, self.ego_vehicle, self.config)

        for _ in range(10):
            sim_stat_list_total.append(sim_long.simulate_state_list(0))

        for sim_state_list in sim_stat_list_total:
            utils_vis.draw_state_list(self.rnd, sim_state_list, 0)
            self.assertEqual(sim_state_list[-1].time_step,
                             self.ego_vehicle.prediction.final_time_step)

        utils_vis.save_fig("test_simulate_mc_long", self.config.general.path_output, 0)

    def test_simulation_lat(self):
        # steering
        sim_lat_left = SimulationLat(Maneuver.STEERLEFT, self.ego_vehicle, self.config)
        simulated_state1 = sim_lat_left.simulate_state_list(0)
        sim_lat_right = SimulationLat(Maneuver.STEERRIGHT, self.ego_vehicle, self.config)
        simulated_state2 = sim_lat_right.simulate_state_list(10)
        self.config.time.steer_width = 2
        sim_lat_left_2 = SimulationLat(Maneuver.STEERLEFT, self.ego_vehicle, self.config)
        simulated_state3 = sim_lat_left_2.simulate_state_list(0)

        # overtaking
        sim_lat_left = SimulationLat(Maneuver.OVERTAKELEFT, self.ego_vehicle, self.config)
        simulated_state4 = sim_lat_left.simulate_state_list(5)
        sim_lat_right = SimulationLat(Maneuver.OVERTAKERIGHT, self.ego_vehicle, self.config)
        simulated_state5 = sim_lat_right.simulate_state_list(15)

        # turning
        sim_lat_left.update_maneuver(Maneuver.TURNLEFT)
        simulated_state6 = sim_lat_left.simulate_state_list(0)
        sim_lat_right.update_maneuver(Maneuver.TURNRIGHT)
        simulated_state7 = sim_lat_right.simulate_state_list(10)

        for i in range(len(simulated_state1)):
            self.assertEqual(simulated_state1[i].time_step, i)
        self.assertEqual(simulated_state1[-1].time_step,
                         self.ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state2[-1].time_step,
                         self.ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state3[-1].time_step,
                         self.ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state4[-1].time_step,
                         self.ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state5[-1].time_step,
                         self.ego_vehicle.prediction.final_time_step)

        for sim_state_list in [simulated_state1, simulated_state2, simulated_state3, simulated_state4,
                               simulated_state5, simulated_state6, simulated_state7]:
            utils_vis.draw_state_list(self.rnd, sim_state_list, 0)

        utils_vis.save_fig("test_simulate_lat", self.config.general.path_output, 0)

    def test_simulation_lat_mc(self):
        sim_stat_list_total = []
        self.config.time.steer_width = 2
        sim_lat_left = SimulationLatMonteCarlo(Maneuver.LANECHANGEMC, self.ego_vehicle, self.config)
        for i in range(10):
            sim_stat_list_total.append(sim_lat_left.simulate_state_list(0))
        sim_lat_left.update_maneuver(Maneuver.TURNMC)
        for i in range(10):
            sim_stat_list_total.append(sim_lat_left.simulate_state_list(0))
        sim_lat_left.update_maneuver(Maneuver.OVERTAKEMC)
        for i in range(10):
            sim_stat_list_total.append(sim_lat_left.simulate_state_list(0))
        for sim_state_list in sim_stat_list_total:
            utils_vis.draw_state_list(self.rnd, sim_state_list, 0)
            self.assertEqual(sim_state_list[-1].time_step,
                             self.ego_vehicle.prediction.final_time_step)

        utils_vis.save_fig("test_simulate_mc_lat", self.config.general.path_output, 0)

    def test_simulation_random(self):
        sim_stat_list_total = []
        sim_lat_left = SimulationRandoMonteCarlo(Maneuver.RANDOMMC, self.ego_vehicle, self.config)
        for i in range(10):
            sim_stat_list_total.append(sim_lat_left.simulate_state_list(0))
            for sim_state_list in sim_stat_list_total:
                utils_vis.draw_state_list(self.rnd, sim_state_list, 0)
                self.assertEqual(sim_state_list[-1].time_step,
                                 self.ego_vehicle.prediction.final_time_step)

            utils_vis.save_fig("test_simulate_mc_random", self.config.general.path_output, 0)

