"""
Unit tests of the module time-scale measures
"""

import unittest

import math

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_crime.measure import TET, TIT, TTCStar, TTB, TTK, TTS, TTR, THW, TTZ, WTTC, TTCE
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_crime.utility.logger as util_logger
from commonroad_crime.utility.simulation import Maneuver

try:
    import commonroad_reach.pycrreach
    from commonroad_crime.measure.time.wttr import WTTR
    module_failed = False
except ImportError:
    module_failed = True


class TestTimeDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Urban-3_3_Repair'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_tet(self):
        self.config.debug.draw_visualization = True
        self.config.debug.save_plots = True
        tet_object_1 = TET(self.config)
        tet_1 = tet_object_1.compute(6)
        tet_object_1.visualize()
        assert math.isclose(tet_1, 2.0, abs_tol=1e-2)

        tet_object_2 = TET(self.config)
        tet_2 = tet_object_2.compute(7)
        assert math.isclose(tet_2, 0.9, abs_tol=1e-2)

    def test_tit(self):
        self.config.debug.draw_visualization = True
        self.config.debug.save_plots = True
        tit_object_1 = TIT(self.config)
        tit_1 = tit_object_1.compute(vehicle_id=6)
        tit_object_1.visualize()
        assert math.isclose(tit_1, 2.012, abs_tol=1e-2)

        tit_object_2 = TIT(self.config)
        tit_2 = tit_object_2.compute(7)
        assert math.isclose(tit_2, 1.40, abs_tol=1e-2)

    def test_ttc(self):
        self.config.debug.draw_visualization = True
        self.config.debug.save_plots = True
        ttc_object_1 = TTCStar(self.config)
        ttc_1 = ttc_object_1.compute()
        ttc_object_1.visualize()
        assert math.isclose(ttc_1, 2.4, abs_tol=1e-2)

        # remove the colliding obstacle
        self.config.scenario.remove_obstacle(self.config.scenario.static_obstacles)
        self.config.update(sce=self.config.scenario)
        ttc_object_2 = TTCStar(self.config)
        ttc_2 = ttc_object_2.compute()
        assert math.isclose(ttc_2, math.inf, abs_tol=1e-2)

        # test scenario with set-based prediction
        self.config.general.name_scenario = "ZAM_Urban-7_1_S-2"
        sce_set, _ = CommonRoadFileReader(self.config.general.path_scenario).\
            open(lanelet_assignment=True)
        self.config.update(ego_id=100, sce=sce_set)
        ttc_object_3 = TTCStar(self.config)
        ttc_3 = ttc_object_3.compute()
        assert math.isclose(ttc_3, 9*sce_set.dt, abs_tol=1e-2)

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
        ttk_object.visualize()
        self.assertEqual(ttk, -math.inf)

        tts_object = TTS(self.config)
        tts = tts_object.compute()
        tts_object.visualize()
        self.assertEqual(tts, 2.2)

        tts2 = tts_object.compute()
        tts_object.visualize()
        self.assertEqual(tts, tts2)

    def test_ttr(self):
        self.config.time.steer_width = 2
        self.config.debug.draw_visualization = True
        ttr_object = TTR(self.config)
        ttr = ttr_object.compute()
        ttr_object.visualize()
        self.assertEqual(ttr, 2.0)
        self.assertEqual(ttr_object.maneuver, Maneuver.BRAKE)

        ttr_2 = ttr_object.compute(10)
        ttr_object.visualize()
        # ttr_2 != ttr - 10 * ttr_object.dt due to the binary search, which might missed some possible solutions
        self.assertGreater(ttr, ttr_2)

        ttr_object.configuration.time.steer_width = 1
        ttr_3 = ttr_object.compute()
        ttr_object.visualize()
        self.assertEqual(ttr_3, 2.2)

        # test scenario with set-based prediction
        self.config.general.name_scenario = "ZAM_Urban-7_1_S-2"
        sce_set, _ = CommonRoadFileReader(self.config.general.path_scenario).\
            open(lanelet_assignment=True)
        self.config.update(ego_id=100, sce=sce_set)
        ttc_object_4 = TTR(self.config)
        ttc_4 = ttc_object_4.compute()
        ttc_object_4.visualize()
        assert math.isclose(ttc_4, 1.25, abs_tol=1e-2)

    def test_thw(self):
        thw_object = THW(self.config)
        thw = thw_object.compute(6, 0)
        thw_object.visualize()
        self.assertEqual(thw, 2.4)

        thw2 = thw_object.compute(6, 10)
        thw_object.visualize()
        self.assertAlmostEqual(thw2, thw - 10 * thw_object.dt)

        thw3 = thw_object.compute(7, 0)
        self.assertEqual(thw3, math.inf)

    def test_wttc(self):
        wttc_object = WTTC(self.config)
        other_obs_id = 6
        wttc = wttc_object.compute(other_obs_id, 0)
        wttc_object.visualize()
        self.assertEqual(wttc, 1.3)

        ttc_object = TTCStar(self.config)
        ttc = ttc_object.compute()
        self.assertGreater(ttc, wttc)

    @unittest.skipIf(module_failed, "No module commonroad_reach installed")
    def test_wttr(self):
        wttr_object = WTTR(self.config)
        wttr = wttr_object.compute(10)
        wttr_object.visualize()
        self.assertEqual(wttr, 1.3)
        wttr2 = wttr_object.compute()
        self.assertAlmostEqual(wttr, wttr2 - 1.)

    def test_ttz(self):
        self.config.general.name_scenario = "ZAM_Zip-2_1_T-1"
        sce_crosswalk, _ = CommonRoadFileReader(self.config.general.path_scenario).\
            open(lanelet_assignment=True)
        self.config.update(ego_id=1, sce=sce_crosswalk)
        ttz_object = TTZ(self.config)
        ttz = ttz_object.compute(0)
        self.assertEqual(ttz, 1.05)
        ttz_object.visualize()

    def test_ttce(self):
        ttce_object = TTCE(self.config)
        ttce = ttce_object.compute(6)
        ttce_object.visualize()
        assert math.isclose(ttce, 2.4, abs_tol=1e-2)

        ttce_object_2 = TTCE(self.config)
        ttce2 = ttce_object_2.compute(7)
        assert math.isclose(ttce2, 2.4, abs_tol=1e-2)

        ttce3 = ttce_object_2.compute(7, time_step=10)
        assert math.isclose(ttce3, ttce2 - 10 * self.config.scenario.dt, abs_tol=1e-2)




