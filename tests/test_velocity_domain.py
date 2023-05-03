"""
Unit tests of the module velocity-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure.velocity.delta_v import DeltaV
import commonroad_crime.utility.logger as util_logger


class TestVelocityDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'DEU_Gar-1_1_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_delta_v(self):
        delta_v_object = DeltaV(self.config)
        delta_v = delta_v_object.compute(0, 202)
        self.assertEqual(delta_v, 13.0)

