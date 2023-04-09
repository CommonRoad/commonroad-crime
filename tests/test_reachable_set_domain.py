"""
Unit tests of the module reachable set scale measures
"""

import unittest

try:
    from commonroad_crime.measure.reachable_set.drivable_area import DA
    from commonroad_crime.data_structure.configuration_builder import ConfigurationBuilder
    import commonroad_crime.utility.logger as util_logger
    module_failed = False
except ImportError:
    module_failed = True


class TestReachableSetDomain(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Urban-3_3_Repair'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()
        self.config.update()

    @unittest.skipIf(module_failed, "No module commonroad_reach installed")
    def test_drivable_area(self):
        da_solver = DA(self.config)
        self.assertAlmostEqual(da_solver.compute(), 60.13)
        da_solver.visualize()
