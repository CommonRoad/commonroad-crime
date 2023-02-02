"""
Unit tests of the module probability-scale measures
"""

import unittest

from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_crime.measure.probability.p_mc import P_MC
import commonroad_crime.utility.logger as util_logger


class TestProbabilityDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'DEU_Gar-1_1_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_p_mc(self):
        p_mc_object = P_MC(self.config)
        p_mc = p_mc_object.compute(0)
        p_mc_object.visualize()
        self.assertLessEqual(abs(p_mc - 0.04), 0.05)
        self.assertLessEqual(p_mc, 1.)
