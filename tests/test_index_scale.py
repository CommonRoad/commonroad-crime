"""
Unit tests of the module index-scale metrics
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.metric.index_scale.btn import BTN
import commonroad_crime.utility.logger as util_logger


class TestIndexScale(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Zip-1_56_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_bnt(self):
        btn_object = BTN(self.config)
        # vehicle in the same lanelet and in front
        btn_1 = btn_object.compute(3, 0)
        btn_object.visualize()
        self.assertEqual(btn_1, 0.09)

        # vehicle in another lanelet
        btn_2 = btn_object.compute(1, 0)
        self.assertEqual(btn_2, 0.0)
