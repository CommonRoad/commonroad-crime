"""
Unit tests of the module reachable set scale metrics
"""

import unittest

from commonroad_crime.metric.reachable_set_scale.drivable_area import DrivableArea
from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_crime.utility.logger as util_logger

try:
    import commonroad_reach.pycrreach
    module_failed = False
except ImportError:
    module_failed = True


class TestSpaceMetrics(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        if module_failed:
            self.skipTest("No module commonroad_reach installed")
        scenario_id = 'DEU_Test-1_1_T-1'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    def test_drivable_area(self):
        da_solver = DrivableArea(self.config)
        self.assertAlmostEqual(da_solver.compute(), 21.1)
        da_solver.visualize()

