"""
Unit tests of the module acceleration-scale metrics
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.metric.acceleration_scale.dst import DST
from commonroad_crime.metric.acceleration_scale.a_long_req import ALongReq
from commonroad_crime.metric.acceleration_scale.a_lat_req import ALatReq
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

    def test_a_long_req(self):
        a_long_req_object = ALongReq(self.config)
        a_long_req_1 = a_long_req_object.compute(202, 0)
        self.assertEqual(a_long_req_1, -0.67)

        a_long_req_2 = a_long_req_object.compute(201, 0)
        self.assertLessEqual(a_long_req_2, 0.0)

        self.config.acceleration_scale.acceleration_mode = 2
        a_long_req_3 = a_long_req_object.compute(202, 0)
        self.assertEqual(a_long_req_3, -5.27)

    def test_a_lat_req(self):
        a_lat_req_object = ALatReq(self.config)
        a_lat_req_1 = a_lat_req_object.compute(202, 0)
