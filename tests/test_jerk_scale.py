"""
Unit tests of the module jerk-scale metrics
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.metric.jerk_scale.lat_j import LatJ
import commonroad_crime.utility.logger as util_logger


class TestJerkScale(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'DEU_Gar-1_1_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_lat_j(self):
        lat_j_obj = LatJ(self.config)
        lat_j_1 = lat_j_obj.compute(0)
        self.assertEqual(lat_j_1, 0.0)

        lat_j_2 = lat_j_obj.compute(20)  # last time step
        self.assertEqual(lat_j_2, None)

