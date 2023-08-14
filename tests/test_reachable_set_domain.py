"""
Unit tests of the module reachable set scale measures
"""

import unittest

from commonroad_crime.measure.reachable_set.drivable_area import DA
from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_crime.utility.logger as util_logger


class TestReachableSetDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = "ZAM_Urban-3_3_Repair"
        self.config = CriMeConfiguration()
        self.config.general.set_scenario_name(scenario_id)
        self.config.vehicle.ego_id = 8
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_drivable_area(self):
        da_solver = DA(self.config)
        self.assertAlmostEqual(da_solver.compute(), 60.13)
        da_solver.visualize()
