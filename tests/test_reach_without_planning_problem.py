"""
Unit tests verifying that reachable-set criticality measures (DA, WTTR) work
both with and without a PlanningProblemSet in the scenario XML or in memory.
"""

import copy
import importlib.util
import os
import sys
import tempfile
import types
import unittest
from unittest.mock import MagicMock

import numpy as np
from omegaconf import OmegaConf
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet


def _ensure_optional_cpp_stubs() -> None:
    """Provide lightweight stubs only when Linux-only C++ bindings are absent."""
    if importlib.util.find_spec("commonroad_dc") is None:
        dc_mod = types.ModuleType("commonroad_dc")
        feas_mod = types.ModuleType("commonroad_dc.feasibility")
        vd_mod = types.ModuleType("commonroad_dc.feasibility.vehicle_dynamics")
        boundary_pkg = types.ModuleType("commonroad_dc.boundary")
        boundary_mod = types.ModuleType("commonroad_dc.boundary.boundary")
        pycrcc_mod = types.ModuleType("commonroad_dc.pycrcc")
        coll_pkg = types.ModuleType("commonroad_dc.collision")
        cd_pkg = types.ModuleType("commonroad_dc.collision.collision_detection")
        disp_mod = types.ModuleType(
            "commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch"
        )
        vd_mod.PointMassDynamics = MagicMock()
        boundary_mod.create_road_boundary_obstacle = MagicMock(
            return_value=(MagicMock(), MagicMock())
        )
        disp_mod.create_collision_checker = MagicMock()
        disp_mod.create_collision_object = MagicMock()
        sys.modules["commonroad_dc"] = dc_mod
        sys.modules["commonroad_dc.feasibility"] = feas_mod
        sys.modules["commonroad_dc.feasibility.vehicle_dynamics"] = vd_mod
        sys.modules["commonroad_dc.boundary"] = boundary_pkg
        sys.modules["commonroad_dc.boundary.boundary"] = boundary_mod
        sys.modules["commonroad_dc.pycrcc"] = pycrcc_mod
        sys.modules["commonroad_dc.collision"] = coll_pkg
        sys.modules["commonroad_dc.collision.collision_detection"] = cd_pkg
        sys.modules[
            "commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch"
        ] = disp_mod

    if importlib.util.find_spec("commonroad_clcs") is None:
        clcs_pkg = types.ModuleType("commonroad_clcs")
        clcs_mod = types.ModuleType("commonroad_clcs.clcs")
        pycrccosy_mod = types.ModuleType("commonroad_clcs.pycrccosy")
        util_mod = types.ModuleType("commonroad_clcs.util")
        pycrccosy_mod.CurvilinearCoordinateSystem = MagicMock()
        pycrccosy_mod.Util = MagicMock()
        clcs_mod.CurvilinearCoordinateSystem = MagicMock()
        util_mod.chaikins_corner_cutting = MagicMock(side_effect=lambda x: x)
        util_mod.resample_polyline = MagicMock(side_effect=lambda x, *a, **k: x)
        util_mod.compute_orientation_from_polyline = MagicMock()
        util_mod.compute_pathlength_from_polyline = MagicMock()
        sys.modules["commonroad_clcs"] = clcs_pkg
        sys.modules["commonroad_clcs.clcs"] = clcs_mod
        sys.modules["commonroad_clcs.pycrccosy"] = pycrccosy_mod
        sys.modules["commonroad_clcs.util"] = util_mod

    if importlib.util.find_spec("commonroad_reach") is None:
        reach_pkg = types.ModuleType("commonroad_reach")
        ds_pkg = types.ModuleType("commonroad_reach.data_structure")
        cb_mod = types.ModuleType(
            "commonroad_reach.data_structure.configuration_builder"
        )
        reach_sub = types.ModuleType("commonroad_reach.data_structure.reach")
        ri_mod = types.ModuleType(
            "commonroad_reach.data_structure.reach.reach_interface"
        )
        pycrreach_mod = types.ModuleType("commonroad_reach.pycrreach")
        util_pkg = types.ModuleType("commonroad_reach.utility")
        vis_mod = types.ModuleType("commonroad_reach.utility.visualization")
        ops_mod = types.ModuleType("commonroad_reach.utility.reach_operation")

        class _StubReachConfig:
            def __init__(self):
                self.general = types.SimpleNamespace(
                    path_scenario="", path_output="", path_scenarios=""
                )
                self.planning = types.SimpleNamespace(
                    steps_computation=10,
                    dt=0.1,
                    coordinate_system="CVLN",
                    CLCS=None,
                )
                self.vehicle = types.SimpleNamespace(
                    ego=types.SimpleNamespace(
                        a_lon_min=-8.0, a_lon_max=8.0, a_lat_min=-4.0, a_lat_max=4.0
                    )
                )
                self.debug = types.SimpleNamespace(save_config=False)
                self.scenario = None
                self.planning_problem = None

            def update(
                self,
                scenario=None,
                planning_problem_set=None,
                planning_problem=None,
                CLCS=None,
                **kwargs,
            ):
                if scenario is None or (
                    planning_problem is None and planning_problem_set is None
                ):
                    raise RuntimeError(
                        "update() must receive both scenario and planning_problem"
                    )
                self.scenario = scenario
                if planning_problem is not None:
                    self.planning_problem = planning_problem
                else:
                    self.planning_problem = list(
                        planning_problem_set.planning_problem_dict.values()
                    )[0]
                self.planning.CLCS = CLCS or MagicMock()

        class _StubBuilder:
            def build_configuration(self, name_scenario):
                return _StubReachConfig()

        class _StubInterface:
            def __init__(self, cfg):
                self.config = cfg
                self.reachable_set = {0: ["node"]}

            def reset(self, cfg):
                self.config = cfg

            def compute_reachable_sets(self, verbose=False):
                pass

            def reachable_set_at_step(self, step):
                return ["node"]

        cb_mod.ConfigurationBuilder = _StubBuilder
        ri_mod.ReachableSetInterface = _StubInterface
        pycrreach_mod.ReachNode = object
        ops_mod.compute_area_of_reach_nodes = lambda nodes: 42.5

        sys.modules["commonroad_reach"] = reach_pkg
        sys.modules["commonroad_reach.data_structure"] = ds_pkg
        sys.modules["commonroad_reach.data_structure.configuration_builder"] = cb_mod
        sys.modules["commonroad_reach.data_structure.reach"] = reach_sub
        sys.modules["commonroad_reach.data_structure.reach.reach_interface"] = ri_mod
        sys.modules["commonroad_reach.pycrreach"] = pycrreach_mod
        sys.modules["commonroad_reach.utility"] = util_pkg
        sys.modules["commonroad_reach.utility.visualization"] = vis_mod
        sys.modules["commonroad_reach.utility.reach_operation"] = ops_mod


_ensure_optional_cpp_stubs()

from commonroad_crime.data_structure.configuration import CriMeConfiguration
from commonroad_crime.measure.reachable_set.drivable_area import DA
import commonroad_crime.utility.general as utils_gen


class TestReachWithoutPlanningProblem(unittest.TestCase):
    def setUp(self) -> None:
        repo_root = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
        self.scenario_path = os.path.join(
            repo_root, "scenarios", "ZAM_Urban-3_3_Repair.xml"
        )
        self.scenario, self.pps = CommonRoadFileReader(self.scenario_path).open(
            lanelet_assignment=True
        )
        self.ego_id = 8
        self.ego_vehicle = self.scenario.obstacle_by_id(self.ego_id)

    def test_structured_omegaconf_schema_remains_valid(self):
        schema = OmegaConf.structured(CriMeConfiguration)
        self.assertIsNotNone(schema)

    def test_preserves_existing_file_planning_problem(self):
        cfg = CriMeConfiguration()
        cfg.general.set_scenario_name("ZAM_Urban-3_3_Repair")
        cfg.update(ego_id=self.ego_id)
        pp = utils_gen.resolve_reach_planning_problem(
            cfg, self.scenario, self.ego_vehicle
        )
        self.assertIsInstance(pp, PlanningProblem)
        expected_id = next(iter(self.pps.planning_problem_dict.keys()))
        self.assertEqual(pp.planning_problem_id, expected_id)

    def test_synthesizes_planning_problem_when_xml_omits_planning_problems(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            sce_copy = copy.deepcopy(self.scenario)
            xml_name = "ZAM_NoPlanningProblem"
            xml_path = os.path.join(tmpdir, f"{xml_name}.xml")
            writer = CommonRoadFileWriter(
                scenario=sce_copy,
                planning_problem_set=PlanningProblemSet(),
                author="test",
                affiliation="test",
                source="test",
            )
            writer.write_to_file(xml_path, OverwriteExistingFile.ALWAYS)

            cfg = CriMeConfiguration()
            cfg.general.path_scenarios = tmpdir + os.sep
            cfg.general.set_scenario_name(xml_name)
            cfg.vehicle.ego_id = self.ego_id
            cfg.update()

            pp = utils_gen.resolve_reach_planning_problem(
                cfg, cfg.scenario, cfg.scenario.obstacle_by_id(self.ego_id)
            )
            self.assertIsInstance(pp, PlanningProblem)
            self.assertEqual(pp.planning_problem_id, self.ego_id)
            np.testing.assert_allclose(
                pp.initial_state.position, self.ego_vehicle.initial_state.position
            )

            da_solver = DA(cfg)
            val = da_solver.compute(time_step=0, verbose=False)
            self.assertTrue(np.isfinite(val))
            self.assertGreater(val, 0.0)

    def test_synthesizes_planning_problem_for_in_memory_scenario_without_xml(self):
        sce_mem = copy.deepcopy(self.scenario)
        sce_mem.scenario_id = "DEU_InMemoryOnly-1_1_T-1"
        cfg = CriMeConfiguration()
        cfg.update(ego_id=self.ego_id, sce=sce_mem)

        pp = utils_gen.resolve_reach_planning_problem(
            cfg, cfg.scenario, cfg.scenario.obstacle_by_id(self.ego_id)
        )
        self.assertIsInstance(pp, PlanningProblem)
        self.assertEqual(pp.planning_problem_id, self.ego_id)


if __name__ == "__main__":
    unittest.main()
