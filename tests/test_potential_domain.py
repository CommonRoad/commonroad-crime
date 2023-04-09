"""
Unit tests of the module potential-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure.potential.pf import PF
import commonroad_crime.utility.logger as util_logger


class TestPotentialDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'DEU_Gar-1_1_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_pf(self):
        pf_object = PF(self.config)
        pf = pf_object.compute(20)
        self.assertEqual(pf, self.config.potential.u_max)
        pf_object.visualize()

