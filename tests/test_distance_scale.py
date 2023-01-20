"""
Unit tests of the module distance-scale metrics
"""

import unittest
import math

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.metric.distance_scale.hw import HW
import commonroad_crime.utility.logger as util_logger


class TestDistanceScale(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Urban-3_3_Repair'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_hw(self):
        hw_object = HW(self.config)
        hw = hw_object.compute(6, 0)
        self.assertGreater(hw, 0)
        self.assertEqual(hw, 20.75)
        hw_object.visualize()
