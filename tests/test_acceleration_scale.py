"""
Unit tests of the module acceleration-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure import DST, ALongReq, ALatReq, AReq
import commonroad_crime.utility.logger as util_logger


class TestAccelerationDomain(unittest.TestCase):
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
        self.assertEqual(dst_2, 24.86)
        dst_object.visualize()

    def test_a_long_req(self):
        a_long_req_object = ALongReq(self.config)
        a_long_req_1 = a_long_req_object.compute(202, 0)
        self.assertEqual(a_long_req_1, -0.81)

        a_long_req_2 = a_long_req_object.compute(201, 0)
        self.assertLessEqual(a_long_req_2, 0.0)

        self.config.acceleration.acceleration_mode = 2
        a_long_req_3 = a_long_req_object.compute(202, 0)
        self.assertEqual(a_long_req_3, -6.57)

    def test_a_lat_req(self):
        a_lat_req_object = ALatReq(self.config)
        a_lat_req_1 = a_lat_req_object.compute(202, 0)
        self.assertEqual(a_lat_req_1, 0.09)

    def test_a_req(self):
        a_req_object = AReq(self.config)
        a_req_1 = a_req_object.compute(202, 0)
        self.assertEqual(a_req_1, 0.81)
