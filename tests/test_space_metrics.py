"""
Unit tests of the module time-to-react computation
"""

import os
import unittest

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_criticality.space.drivable_area import DrivableAreaCriticality
from commonroad_criticality.utility.configuration_builder import ConfigurationBuilder


class TestSpaceMetrics(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        root_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")
        self.scenario_root_path = os.path.join(root_path, "scenarios")

        root_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")
        ConfigurationBuilder.set_root_path(root_path)

        scenario_id = 'DEU_Test-1_1_T-1'
        config = ConfigurationBuilder.build_configuration(scenario_id)

        self.scenario, _ = CommonRoadFileReader(config.config_general.path_scenario).open(lanelet_assignment=True)
        self.drivable_criticality = DrivableAreaCriticality(self.scenario,
                                                            id_vehicle=0,
                                                            config=config)

    def test_generation(self):
        criticality = self.drivable_criticality.compute()
        self.assertGreaterEqual(criticality, 0)
        self.assertLessEqual(criticality, 1)
