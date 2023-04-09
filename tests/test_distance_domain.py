"""
Unit tests of the module distance-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure import HW, DCE
import commonroad_crime.utility.logger as util_logger


class TestDistanceDomain(unittest.TestCase):
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

    def test_dce(self):
        dce_object_1 = DCE(self.config)
        dce_1 = dce_object_1.compute(6, 0)
        dce_object_1.visualize()
        self.assertEqual(dce_1, 0.00)

        dce_object_2 = DCE(self.config)
        dce_2 = dce_object_2.compute(7, 0)
        dce_object_2.visualize()
        self.assertEqual(dce_2, 0.98)
