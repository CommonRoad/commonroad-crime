"""
Unit tests of the module time-scale metrics
"""

import unittest
import math

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_crime.utility.logger as util_logger
from commonroad_crime.metric.time_scale.ttc_star import TTCStar
from commonroad_crime.metric.time_scale.ttb import TTB
from commonroad_crime.metric.time_scale.ttk import TTK
from commonroad_crime.metric.time_scale.tts import TTS
from commonroad_crime.metric.time_scale.ttr import TTR
from commonroad_crime.metric.time_scale.thw import THW
from commonroad_crime.metric.time_scale.wttc import WTTC
from commonroad_crime.utility.simulation import Maneuver

try:
    import commonroad_reach.pycrreach
    from commonroad_crime.metric.time_scale.wttr import WTTR
    module_failed = False
except ImportError:
    module_failed = True


class TestTimeScale(unittest.TestCase):
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

    def test_ttm(self):
        self.config.debug.draw_visualization = True
        ttb_object = TTB(self.config)
        ttb = ttb_object.compute()
        ttb_object.visualize()
        self.assertEqual(ttb, 1.6)
        ttb2 = ttb_object.compute()
        self.assertEqual(ttb, ttb2)

        ttk_object = TTK(self.config)
        ttk = ttk_object.compute()
        self.assertEqual(ttk, 0.6)

        tts_object = TTS(self.config)
        tts = tts_object.compute()
        tts_object.visualize()
        self.assertEqual(tts, 2.2)

        tts2 = tts_object.compute()
        tts_object.visualize()
        self.assertEqual(tts, tts2)

    def test_ttr(self):
        self.config.time_scale.steer_width = 2
        self.config.debug.draw_visualization = True
        ttr_object = TTR(self.config)
        ttr = ttr_object.compute()
        ttr_object.visualize()
        self.assertEqual(ttr, 1.6)
        self.assertEqual(ttr_object.maneuver, Maneuver.BRAKE)

        ttr_2 = ttr_object.compute(10)
        ttr_object.visualize()
        # ttr_2 != ttr - 10 * ttr_object.dt due to the binary search, which might missed some possible solutions
        self.assertGreater(ttr, ttr_2)

        ttr_object.configuration.time_scale.steer_width = 1
        ttr_3 = ttr_object.compute()
        ttr_object.visualize()
        self.assertEqual(ttr_3, 2.2)

    def test_thw(self):
        thw_object = THW(self.config)
        thw = thw_object.compute(6, 0)
        thw_object.visualize()
        self.assertEqual(thw, 2.9)

        thw2 = thw_object.compute(6, 10)
        thw_object.visualize()
        self.assertEqual(thw2, thw - 10 * thw_object.dt)

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
        wttr = wttr_object.compute(0)
        wttr_object.visualize()
        self.assertEqual(wttr, 2.2)





