"""
Unit tests of the module acceleration-scale metrics
"""

import unittest
import math

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.metric.acceleration_scale.dst import DST
import commonroad_crime.utility.logger as util_logger


class TestAccelerationScale(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'DEU_Gar-1_1_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_dst(self):
        dst_object = DST(self.config)
        dst_1 = dst_object.compute(201, 0)
        dst_2 = dst_object.compute(202, 0)
        self.assertEqual(dst_1, 0.)
        self.assertEqual(dst_2, 7.75)
        dst_object.visualize()
