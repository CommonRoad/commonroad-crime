"""
Unit tests of the module time metrics
"""

import unittest
import math

from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_crime.utility.logger as util_logger
from commonroad_crime.metric.time_scale.ttc import TTC
from commonroad_crime.metric.time_scale.ttb import TTB
from commonroad_crime.metric.time_scale.ttk import TTK
from commonroad_crime.metric.time_scale.tts import TTS
from commonroad_crime.metric.time_scale.ttr import TTR
from commonroad_crime.metric.time_scale.thw import THW
from commonroad_crime.metric.time_scale.wttc import WTTC
from commonroad_crime.metric.time_scale.wttr import WTTR
from commonroad_crime.utility.simulation import SimulationLong, SimulationLat, Maneuver
import commonroad_crime.utility.visualization as utils_vis


class TestTimeMetrics(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Urban-3_3_Repair'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_ttc(self):
        self.config.debug.draw_visualization = True
        self.config.debug.save_plots = True
        ttc_object_1 = TTC(self.config)
        ttc_1 = ttc_object_1.compute()
        ttc_object_1.visualize()
        assert math.isclose(ttc_1, 2.4, abs_tol=1e-2)

        # remove the colliding obstacle
        self.config.scenario.remove_obstacle(self.config.scenario.static_obstacles)
        self.config.update(sce=self.config.scenario)
        ttc_object_2 = TTC(self.config)
        ttc_2 = ttc_object_2.compute()
        assert math.isclose(ttc_2, math.inf, abs_tol=1e-2)

    def test_simulation_long(self):
        ego_vehicle = self.config.scenario.obstacle_by_id(self.config.vehicle.ego_id)

        rnd = MPRenderer()
        self.config.scenario.draw(rnd)
        rnd.render()

        sim_long = SimulationLong(Maneuver.BRAKE, ego_vehicle, self.config)
        simulated_state1 = sim_long.simulate_state_list(0, rnd)
        for i in range(len(simulated_state1)):
            self.assertEqual(simulated_state1[i].time_step, i)
        self.assertEqual(
            simulated_state1[-1].time_step,
            ego_vehicle.prediction.final_time_step)
        self.assertEqual(sim_long.check_velocity_feasibility(
            simulated_state1[-1]), True)

        sim_long.update_maneuver(Maneuver.KICKDOWN)
        simulated_state2 = sim_long.simulate_state_list(20, rnd)
        self.assertEqual(sim_long.check_velocity_feasibility(
            simulated_state2[-1]), True)

        sim_long.update_maneuver(Maneuver.CONSTANT)
        simulated_state3 = sim_long.simulate_state_list(10, rnd)
        self.assertEqual(sim_long.check_velocity_feasibility(
            simulated_state3[-1]), True)
        utils_vis.save_fig("test_simulate_long", self.config.general.path_output, 0)

    def test_simulation_lat(self):
        self.config.debug.draw_visualization = True
        ego_vehicle = self.config.scenario.obstacle_by_id(self.config.vehicle.ego_id)

        rnd = MPRenderer()
        self.config.scenario.draw(rnd)
        rnd.render()

        sim_lat_left = SimulationLat(Maneuver.STEERLEFT, ego_vehicle, self.config)
        simulated_state1 = sim_lat_left.simulate_state_list(0, rnd)
        sim_lat_right = SimulationLat(Maneuver.STEERRIGHT, ego_vehicle, self.config)
        simulated_state2 = sim_lat_right.simulate_state_list(10, rnd)
        self.config.time_scale.steer_width = 2
        sim_lat_left_2 = SimulationLat(Maneuver.STEERLEFT, ego_vehicle, self.config)
        simulated_state3 = sim_lat_left_2.simulate_state_list(0, rnd)

        for i in range(len(simulated_state1)):
            self.assertEqual(simulated_state1[i].time_step, i)
        self.assertEqual(simulated_state1[-1].time_step,
                         ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state2[-1].time_step,
                         ego_vehicle.prediction.final_time_step)
        self.assertEqual(simulated_state3[-1].time_step,
                         ego_vehicle.prediction.final_time_step)

        utils_vis.save_fig("test_simulate_lat", self.config.general.path_output, 0)

    def test_ttm(self):
        self.config.debug.draw_visualization = True
        ttb_object = TTB(self.config)
        ttb = ttb_object.compute()
        ttb_object.visualize()
        self.assertEqual(ttb, 2.0)
        ttb2 = ttb_object.compute()
        self.assertEqual(ttb, ttb2)

        ttk_object = TTK(self.config)
        ttk = ttk_object.compute()
        self.assertEqual(ttk, 0.7)

        tts_object = TTS(self.config)
        tts = tts_object.compute()
        self.assertEqual(tts, 1.7)
        tts_object.visualize()

        tts2 = tts_object.compute()
        tts_object.visualize()
        self.assertEqual(tts, tts2)

    def test_ttr(self):
        self.config.time_scale.steer_width = 2
        self.config.debug.draw_visualization = True
        ttr_object = TTR(self.config)
        ttr = ttr_object.compute()
        ttr_object.visualize()
        self.assertEqual(ttr, 2.0)
        self.assertEqual(ttr_object.maneuver, Maneuver.BRAKE)

        ttr2 = ttr_object.compute(10)
        ttr_object.visualize()
        self.assertEqual(ttr2, ttr - 10 * ttr_object.dt)

    def test_thw(self):
        thw_object = THW(self.config)
        other_obs_id = 6
        thw = thw_object.compute(other_obs_id, 0)
        thw_object.visualize()
        self.assertEqual(thw, 2.9)

        thw2 = thw_object.compute(other_obs_id, 10)
        thw_object.visualize()
        self.assertEqual(thw2, thw - 10 * thw_object.dt)

    def test_wttc(self):
        wttc_object = WTTC(self.config)
        other_obs_id = 6
        wttc = wttc_object.compute(other_obs_id, 0)
        wttc_object.visualize()
        self.assertEqual(wttc, 1.3)

        ttc_object = TTC(self.config)
        ttc = ttc_object.compute()
        self.assertGreater(ttc, wttc)

    def test_wttr(self):
        wttr_object = WTTR(self.config)
        wttr = wttr_object.compute(0)
        wttr_object.visualize()
        self.assertEqual(wttr, 2.0)





