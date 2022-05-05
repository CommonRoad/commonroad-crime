"""
Unit tests of the module time-to-react computation
"""

import os
import unittest
import math

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_criticality.time.ttr import TTR
from commonroad_criticality.time.simulation import CutOffAction

from stl_crmonitor.crmonitor.common.world_state import WorldState


class TestTTR(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        root_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")
        self.scenario_root_path = os.path.join(root_path, "scenarios")
        scenario_file = os.path.join(self.scenario_root_path, "ZAM_Urban-3_3_Repair.xml")
        self.scenario, _ = CommonRoadFileReader(scenario_file).open(lanelet_assignment=True)
        ego_id = 8
        self.world_state = WorldState.create_from_scenario(self.scenario, ego_id)
        self.ttr_object = TTR(self.world_state)

    def test_ttc(self):
        ttc = self.ttr_object.ttc
        assert math.isclose(ttc, 2.4, abs_tol=1e-2)

    def test_ttr_1(self):
        maneuver_set = [CutOffAction.STEERLEFT,
                        CutOffAction.BRAKE,
                        CutOffAction.KICKDOWN,
                        CutOffAction.STEERRIGHT]  # all maneuvers
        ttr = self.ttr_object.generate(maneuver_set)
        assert math.isclose(ttr, 2.3, abs_tol=1e-2)

    def test_ttr_2(self):
        maneuver_set = [CutOffAction.STEERRIGHT]  # impossible maneuver
        ttr = self.ttr_object.generate(maneuver_set)
        assert math.isclose(ttr, -math.inf, abs_tol=1e-2)