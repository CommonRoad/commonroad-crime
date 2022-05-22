"""
Unit tests of the module space metrics
"""

import os
import unittest

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_criticality.space.drivable_area import DrivableAreaCriticality
from commonroad_criticality.utility.configuration_builder import ConfigurationBuilder
import commonroad_criticality.utility.logger as util_logger


class TestSpaceMetrics(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'DEU_Test-1_1_T-1'
        config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(config)
        config.print_configuration_summary()

        self.scenario, _ = CommonRoadFileReader(config.general.path_scenario).open(lanelet_assignment=True)
        self.drivable_criticality = DrivableAreaCriticality(self.scenario,
                                                            id_vehicle=0,
                                                            config=config)

    def test_generation(self):
        criticality = self.drivable_criticality.compute()
        self.assertGreaterEqual(criticality, 0)
        self.assertLessEqual(criticality, 1)
