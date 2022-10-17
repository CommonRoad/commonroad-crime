"""
Unit tests of the module time metrics
"""

import unittest
import math

from commonroad_criticality.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_criticality.data_structure.logger as util_logger
from commonroad_criticality.metric.time_scale.ttc import TTC


class TestTimeMetrics(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        scenario_id = 'ZAM_Urban-3_3_Repair'
        self.config = ConfigurationBuilder.build_configuration(scenario_id)
        util_logger.initialize_logger(self.config)
        self.config.print_configuration_summary()

    def test_ttc(self):
        self.config.debug.draw_visualization = False
        self.config.debug.save_plots = True
        self.config.update()
        ttc_object_1 = TTC(self.config)
        ttc_1 = ttc_object_1.compute()
        assert math.isclose(ttc_1, 2.4, abs_tol=1e-2)

        # remove the colliding obstacle
        self.config.scenario.remove_obstacle(self.config.scenario.static_obstacles)
        self.config.update(sce=self.config.scenario)
        ttc_object_2 = TTC(self.config)
        ttc_2 = ttc_object_2.compute()
        assert math.isclose(ttc_2, math.inf, abs_tol=1e-2)

