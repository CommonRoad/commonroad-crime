"""
Unit tests of the module time-to-react computation
"""

import os
import unittest
import math

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_criticality.space.drivable_area import DrivableAreaCriticality
from commonroad_criticality.utility.configuration_builder import ConfigurationBuilder
from stl_crmonitor.crmonitor.common.world_state import WorldState


class TestSpaceMetrics(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        root_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")
        self.scenario_root_path = os.path.join(root_path, "scenarios")
        scenario_id = 'DEU_Test-1_1_T-1'
        scenario_file = os.path.join(self.scenario_root_path, scenario_id + ".xml")
        self.scenario, _ = CommonRoadFileReader(scenario_file).open(lanelet_assignment=True)
        ConfigurationBuilder.set_root_path(root_path)
        config = ConfigurationBuilder.build_configuration(scenario_id)
        self.drivable_criticality = DrivableAreaCriticality(self.scenario,
                                                            id_vehicle=0,
                                                            config=config)

    def test_generation(self):
        self.drivable_criticality.compute()
        pass