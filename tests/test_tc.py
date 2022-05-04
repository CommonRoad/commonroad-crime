"""
Unit tests of the module time-to-comply computation
"""

import os
import unittest
import math

from commonroad.common.file_reader import CommonRoadFileReader
from stl_crmonitor.crmonitor.common.world_state import WorldState

from commonroad_criticality.tc import TC
from commonroad_criticality.simulation import SimulationLong, SimulationLateral, CutOffAction
from commonroad_criticality.utils import check_velocity_feasibility, update_ego_vehicle
from commonroad_criticality.monitor_wrapper import STLRuleMonitor


class TestTC(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()
        root_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")
        self.scenario_root_path = os.path.join(root_path, "scenarios")
        scenario_file = os.path.join(self.scenario_root_path, "DEU_test_safe_distance.xml")
        self.scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open(lanelet_assignment=True)
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
        self.ego_id = 1003
        self.rule_monitor = STLRuleMonitor(self.scenario,
                                           planning_problem,
                                           self.ego_id,
                                           ["R_G1"])

    def test_tv(self):
        tc_object = TC(self.rule_monitor)
        assert math.isclose(tc_object.tv,
                            2.0,
                            abs_tol=1e-2)

    def test_simulation_long(self):
        ego_vehicle = self.scenario.obstacle_by_id(self.ego_id)
        sim_long = SimulationLong(CutOffAction.BRAKE, ego_vehicle, 0, dt=self.scenario.dt)
        simulated_state1 = sim_long.simulate_state_list()
        self.assertEqual(
            simulated_state1[-1].time_step,
            50)
        self.assertEqual(check_velocity_feasibility(
            simulated_state1[-1],
            sim_long.parameters),
            True)
        sim_long.action = CutOffAction.STEADYSPEED
        simulated_state2 = sim_long.simulate_state_list()
        self.assertEqual(
            simulated_state2[-1].time_step,
            50)
        sim_long.action = CutOffAction.KICKDOWN
        simulated_state3 = sim_long.simulate_state_list()
        self.assertEqual(
            simulated_state3[-1].time_step,
            50)
        self.assertEqual(
            check_velocity_feasibility(
                simulated_state3[-1],
                sim_long.parameters),
            True)

    def test_simulate_lateral(self):
        ego_vehicle = self.scenario.obstacle_by_id(self.ego_id)
        world_state = self.rule_monitor.world_state
        sim_lat = SimulationLateral(
            CutOffAction.LANECHANGELEFT,
            ego_vehicle,
            0,
            world_state,
            dt=world_state.dt)
        simulated_state_list1 = sim_lat.simulate_state_list()
        final_lanelet = self.scenario.lanelet_network.find_lanelet_by_position(
            [simulated_state_list1[-1].position])[0]
        final_lane = world_state.road_network.find_lane_by_lanelet(final_lanelet[0])
        self.assertEqual(
            world_state.ego_vehicle.lane.adj_left.lane_id,
            final_lane.lane_id)
        sim_lat.action = CutOffAction.LANECHANGERIGHT
        simulated_state_list2 = sim_lat.simulate_state_list()
        final_lanelet = self.scenario.lanelet_network.find_lanelet_by_position(
            [simulated_state_list2[-1].position])[0]
        final_lane = world_state.road_network.find_lane_by_lanelet(final_lanelet[0])
        self.assertEqual(
            world_state.ego_vehicle.lane.adj_right.lane_id,
            final_lane.lane_id)

    def test_tc_1(self):
        tc_object = TC(self.rule_monitor)
        tc = tc_object.generate([CutOffAction.LANECHANGELEFT])
        self.assertEqual(
            tc,
            -math.inf)

    def test_tc_2(self):
        tc_object = TC(self.rule_monitor)
        tc = tc_object.generate([CutOffAction.LANECHANGERIGHT])
        self.assertEqual(
            round(tc, 1),
            .5)

    def test_tc_3(self):
        tc_object = TC(self.rule_monitor)
        tc = tc_object.generate([CutOffAction.BRAKE])
        self.assertEqual(
            round(tc, 1),
            1.9)

    def test_tc_total(self):
        tc_object = TC(self.rule_monitor)
        tc = tc_object.generate([CutOffAction.LANECHANGELEFT,
                                 CutOffAction.LANECHANGERIGHT,
                                 CutOffAction.KICKDOWN,
                                 CutOffAction.BRAKE])
        self.assertEqual(
            round(tc, 1),
            1.9)
        self.assertEqual(
            tc_object.compliant_maneuver, CutOffAction.BRAKE
        )

    def test_update_world_state(self):
        # simulate a new trajectory of the ego vehicle
        ego_vehicle = self.scenario.obstacle_by_id(self.ego_id)
        world_state = self.rule_monitor.world_state
        sim_long = SimulationLong(CutOffAction.BRAKE, ego_vehicle, 0, dt=world_state.dt)
        new_state_list = sim_long.simulate_state_list()
        # 1. directly update the ego vehicle
        update_ego_vehicle(world_state.road_network,
                           world_state.ego_vehicle,
                           new_state_list,
                           0,
                           world_state.dt)
        # 2. recreate the world state
        ego_vehicle.prediction.trajectory.state_list = new_state_list
        world_state_updated = WorldState.create_from_scenario(self.scenario, self.ego_id)
        # comparison
        # ---> length of the state list
        self.assertEqual(
            len(world_state.ego_vehicle.states_cr),
            len(world_state_updated.ego_vehicle.states_cr),
        )
        self.assertEqual(
            len(world_state.ego_vehicle.states_lon),
            len(world_state_updated.ego_vehicle.states_lon),
        )
        self.assertEqual(
            len(world_state.ego_vehicle.states_lat),
            len(world_state_updated.ego_vehicle.states_lat),
        )
        # ---> check the final state
        # whether its the same
        self.assertTrue(world_state.ego_vehicle.state_list_cr[-1] ==
                        world_state_updated.ego_vehicle.state_list_cr[-1])
        self.assertTrue(world_state.ego_vehicle.states_lat[ego_vehicle.prediction.final_time_step].d ==
                        world_state_updated.ego_vehicle.states_lat[ego_vehicle.prediction.final_time_step].d)
        self.assertTrue(world_state.ego_vehicle.states_lon[ego_vehicle.prediction.final_time_step].s ==
                        world_state_updated.ego_vehicle.states_lon[ego_vehicle.prediction.final_time_step].s)
        # ---> check the lane
        self.assertEqual(world_state.ego_vehicle.lane.contained_lanelets,
                         world_state_updated.ego_vehicle.lane.contained_lanelets)