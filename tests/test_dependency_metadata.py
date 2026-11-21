"""
Unit tests verifying PEP 517 build metadata, Python version bounds, and
post-commonroad_clcs-split dependency specifier compatibility.
"""

import ast
import configparser
import importlib.util
import os
import sys
import types
import unittest
from unittest.mock import MagicMock

from omegaconf import OmegaConf
from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet
from packaging.version import Version


def _ensure_optional_cpp_stubs() -> None:
    """Provide lightweight stubs only when Linux-only C++ bindings are absent."""
    if importlib.util.find_spec("commonroad_dc") is None:
        dc_mod = types.ModuleType("commonroad_dc")
        feas_mod = types.ModuleType("commonroad_dc.feasibility")
        vd_mod = types.ModuleType("commonroad_dc.feasibility.vehicle_dynamics")
        vd_mod.PointMassDynamics = MagicMock()
        sys.modules["commonroad_dc"] = dc_mod
        sys.modules["commonroad_dc.feasibility"] = feas_mod
        sys.modules["commonroad_dc.feasibility.vehicle_dynamics"] = vd_mod

    if importlib.util.find_spec("commonroad_clcs") is None:
        clcs_pkg = types.ModuleType("commonroad_clcs")
        clcs_mod = types.ModuleType("commonroad_clcs.clcs")
        pycrccosy_mod = types.ModuleType("commonroad_clcs.pycrccosy")
        util_mod = types.ModuleType("commonroad_clcs.util")

        class _BaseCLCS:
            pass

        class _WrapperCLCS(_BaseCLCS):
            pass

        pycrccosy_mod.CurvilinearCoordinateSystem = _BaseCLCS
        pycrccosy_mod.Util = MagicMock()
        clcs_mod.CurvilinearCoordinateSystem = _WrapperCLCS
        util_mod.chaikins_corner_cutting = MagicMock(side_effect=lambda x: x)
        util_mod.resample_polyline = MagicMock(side_effect=lambda x, *a, **k: x)
        util_mod.compute_orientation_from_polyline = MagicMock()
        util_mod.compute_pathlength_from_polyline = MagicMock()
        sys.modules["commonroad_clcs"] = clcs_pkg
        sys.modules["commonroad_clcs.clcs"] = clcs_mod
        sys.modules["commonroad_clcs.pycrccosy"] = pycrccosy_mod
        sys.modules["commonroad_clcs.util"] = util_mod


_ensure_optional_cpp_stubs()

from commonroad_crime.data_structure import configuration as cfg_module
from commonroad_crime.data_structure.configuration import CriMeConfiguration
import commonroad_clcs.pycrccosy as pycrccosy


def _parse_setup_kwargs(setup_path: str) -> dict:
    with open(setup_path, "r", encoding="utf-8") as fh:
        tree = ast.parse(fh.read(), filename=setup_path)
    out = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "setup":
            for kw in node.keywords:
                if kw.arg in ("python_requires", "install_requires"):
                    out[kw.arg] = ast.literal_eval(kw.value)
    return out


class TestDependencyMetadata(unittest.TestCase):
    def setUp(self) -> None:
        self.repo_root = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
        self.setup_path = os.path.join(self.repo_root, "setup.py")
        self.pyproject_path = os.path.join(self.repo_root, "pyproject.toml")

    def test_pyproject_pep517_build_backend(self):
        self.assertTrue(os.path.isfile(self.pyproject_path))
        parser = configparser.ConfigParser()
        parser.read(self.pyproject_path, encoding="utf-8")
        self.assertIn("build-system", parser)
        backend = ast.literal_eval(parser["build-system"]["build-backend"])
        requires = ast.literal_eval(parser["build-system"]["requires"])
        self.assertEqual(backend, "setuptools.build_meta")
        self.assertTrue(any("setuptools" in req for req in requires))

    def test_python_requires_supports_310_and_311(self):
        meta = _parse_setup_kwargs(self.setup_path)
        spec = SpecifierSet(meta["python_requires"])
        self.assertIn(Version("3.10.0"), spec)
        self.assertIn(Version("3.11.9"), spec)
        self.assertNotIn(Version("3.9.18"), spec)
        self.assertNotIn(Version("3.12.0"), spec)

    def test_commonroad_dependencies_resolve_post_clcs_split_versions(self):
        meta = _parse_setup_kwargs(self.setup_path)
        reqs = {
            Requirement(r).name: Requirement(r).specifier
            for r in meta["install_requires"]
        }

        # Verify compatibility with commonroad-reach 2025.2.1's transitive pins
        self.assertIn(Version("2025.2.1"), reqs["commonroad-reach"])
        self.assertIn(Version("2025.4.0"), reqs["commonroad-drivability-checker"])
        self.assertIn(Version("2025.2.0"), reqs["commonroad-clcs"])
        self.assertIn(Version("2025.1.0"), reqs["commonroad-route-planner"])
        self.assertIn(Version("2024.3"), reqs["commonroad-io"])
        self.assertIn(Version("2026.1"), reqs["commonroad-io"])

    def test_clcs_type_harmonized_and_default_attribute_present(self):
        self.assertIs(
            cfg_module.CurvilinearCoordinateSystem,
            pycrccosy.CurvilinearCoordinateSystem,
        )
        schema = OmegaConf.structured(CriMeConfiguration)
        self.assertIsNotNone(schema)
        cfg = CriMeConfiguration()
        self.assertIsNone(cfg.vehicle.curvilinear.clcs)


if __name__ == "__main__":
    unittest.main()
