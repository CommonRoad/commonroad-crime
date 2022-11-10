"""
Unit tests of the utility functions for simulation
"""

import unittest

from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_crime.utility.logger as util_logger

from commonroad_crime.utility.simulation import SimulationLong, SimulationLat, Maneuver
import commonroad_crime.utility.visualization as utils_vis


class TestSimulation(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Urban-3_3_Repair'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_simulation_long(self):
        ego_vehicle = self.config.scenario.obstacle_by_id(self.config.vehicle.ego_id)

        rnd = MPRenderer()
        self.config.scenario.draw(rnd)
        rnd.render()

        sim_long = SimulationLong(Maneuver.BRAKE, ego_vehicle, self.config)
        simulated_state1 = sim_long.simulate_state_list(0)
        for i in range(len(simulated_state1)):
            self.assertEqual(simulated_state1[i].time_step, i)
        self.assertEqual(
            simulated_state1[-1].time_step,
            ego_vehicle.prediction.final_time_step)
        self.assertEqual(sim_long.check_velocity_feasibility(
            simulated_state1[-1]), True)

        sim_long.update_maneuver(Maneuver.KICKDOWN)
        simulated_state2 = sim_long.simulate_state_list(20)
        self.assertEqual(sim_long.check_velocity_feasibility(
            simulated_state2[-1]), True)

        sim_long.update_maneuver(Maneuver.CONSTANT)
        simulated_state3 = sim_long.simulate_state_list(10)
        self.assertEqual(sim_long.check_velocity_feasibility(
            simulated_state3[-1]), True)

        utils_vis.draw_state_list(rnd, simulated_state1, 0)
        utils_vis.draw_state_list(rnd, simulated_state2, 20)
        utils_vis.draw_state_list(rnd, simulated_state3, 10)
        utils_vis.save_fig("test_simulate_long", self.config.general.path_output, 0)

    def test_simulation_lat(self):
        self.config.debug.draw_visualization = True
        ego_vehicle = self.config.scenario.obstacle_by_id(self.config.vehicle.ego_id)

        rnd = MPRenderer()
        self.config.scenario.draw(rnd)
        rnd.render()

        # steering
        sim_lat_left = SimulationLat(Maneuver.STEERLEFT, ego_vehicle, self.config)
        simulated_state1 = sim_lat_left.simulate_state_list(0)
        sim_lat_right = SimulationLat(Maneuver.STEERRIGHT, ego_vehicle, self.config)
        simulated_state2 = sim_lat_right.simulate_state_list(10)
        self.config.time_scale.steer_width = 2
        sim_lat_left_2 = SimulationLat(Maneuver.STEERLEFT, ego_vehicle, self.config)
        simulated_state3 = sim_lat_left_2.simulate_state_list(0)

        # overtaking
        sim_lat_left = SimulationLat(Maneuver.OVERTAKELEFT, ego_vehicle, self.config)
        simulated_state4 = sim_lat_left.simulate_state_list(5)
        sim_lat_right = SimulationLat(Maneuver.OVERTAKERIGHT, ego_vehicle, self.config)
        simulated_state5 = sim_lat_right.simulate_state_list(15)

        # turning
        sim_lat_left.update_maneuver(Maneuver.TURNLEFT)
        simulated_state6 = sim_lat_left.simulate_state_list(0)
        sim_lat_right.update_maneuver(Maneuver.TURNRIGHT)
        simulated_state7 = sim_lat_right.simulate_state_list(10)

        for i in range(len(simulated_state1)):
            self.assertEqual(simulated_state1[i].time_step, i)
        self.assertEqual(simulated_state1[-1].time_step,
                         ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state2[-1].time_step,
                         ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state3[-1].time_step,
                         ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state4[-1].time_step,
                         ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state5[-1].time_step,
                         ego_vehicle.prediction.final_time_step)

        utils_vis.draw_state_list(rnd, simulated_state1, 0)
        utils_vis.draw_state_list(rnd, simulated_state2, 10)
        utils_vis.draw_state_list(rnd, simulated_state3, 0)
        utils_vis.draw_state_list(rnd, simulated_state4, 5)
        utils_vis.draw_state_list(rnd, simulated_state5, 15)
        utils_vis.draw_state_list(rnd, simulated_state6, 0)
        utils_vis.draw_state_list(rnd, simulated_state7, 10)

        utils_vis.save_fig("test_simulate_lat", self.config.general.path_output, 0)




