"""
Unit tests of the module time metrics
"""

import unittest
import math

from commonroad_criticality.data_structure.configuration_builder import ConfigurationBuilder
import commonroad_criticality.data_structure.logger as util_logger
from commonroad_criticality.metric.time_scale.ttc import TTC
from commonroad_criticality.utility.simulation import SimulationLong, Maneuver


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

    def test_simulation_long(self):
        self.config.update()
        ego_vehicle = self.config.scenario.obstacle_by_id(self.config.vehicle.ego_id)
        sim_long = SimulationLong(Maneuver.BRAKE, ego_vehicle, self.config)
        simulated_state1 = sim_long.simulate_state_list(0)
        self.assertEqual(
            simulated_state1[-1].time_step,
            ego_vehicle.prediction.final_time_step)
        self.assertEqual(sim_long.check_velocity_feasibility(
            simulated_state1[-1]), True)

        sim_long.update_maneuver(Maneuver.KICKDOWN)
        simulated_state2 = sim_long.simulate_state_list(0)
        self.assertEqual(sim_long.check_velocity_feasibility(
            simulated_state2[-1]), True)

        sim_long.update_maneuver(Maneuver.CONSTANT)
        simulated_state3 = sim_long.simulate_state_list(10)
        self.assertEqual(simulated_state3[10].velocity,
                         math.sqrt(simulated_state3[-1].velocity**2 + simulated_state3[-1].velocity_y**2))



