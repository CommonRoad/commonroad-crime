"""
Unit tests of the module jerk-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure import LatJ, LongJ
import commonroad_crime.utility.logger as util_logger


class TestJerkDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Zip-1_56_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_lat_j(self):
        lat_j_obj = LatJ(self.config)
        lat_j_1 = lat_j_obj.compute(0)
        self.assertEqual(lat_j_1, 0.0)

        lat_j_2 = lat_j_obj.compute(40)
        self.assertEqual(lat_j_2, 0.0)

        lat_j_3 = lat_j_obj.compute(85)
        self.assertNotEqual(lat_j_3, None)  # last time step

        lat_j_4 = lat_j_obj.compute(84)
        self.assertEqual(lat_j_4, 0.0)  # time step before the last one

        lat_j_5 = lat_j_obj.compute(83)
        self.assertEqual(lat_j_5, 0.0)  # two time steps before the last one

    def test_long_j(self):
        long_j_obj = LongJ(self.config)
        long_j_1 = long_j_obj.compute(0)
        self.assertEqual(long_j_1, 0.0)

        long_j_2 = long_j_obj.compute(40)
        self.assertEqual(long_j_2, 1.71)
