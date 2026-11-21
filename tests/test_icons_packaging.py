"""
Unit tests verifying that icon assets are shipped inside the installed package.
"""

import ast
import importlib.util
import os
import sys
import types
import unittest
from unittest.mock import MagicMock

import matplotlib.pyplot as plt


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
        pycrccosy_mod.CurvilinearCoordinateSystem = MagicMock()
        clcs_mod.CurvilinearCoordinateSystem = MagicMock()
        util_mod.chaikins_corner_cutting = MagicMock(side_effect=lambda x: x)
        util_mod.resample_polyline = MagicMock(side_effect=lambda x, *a, **k: x)
        util_mod.compute_orientation_from_polyline = MagicMock()
        util_mod.compute_pathlength_from_polyline = MagicMock()
        sys.modules["commonroad_clcs"] = clcs_pkg
        sys.modules["commonroad_clcs.clcs"] = clcs_mod
        sys.modules["commonroad_clcs.pycrccosy"] = pycrccosy_mod
        sys.modules["commonroad_clcs.util"] = util_mod


_ensure_optional_cpp_stubs()

import commonroad_crime
from commonroad_crime.data_structure.configuration import GeneralConfiguration


class TestIconsPackaging(unittest.TestCase):
    def test_default_path_icons_points_inside_installed_package(self):
        cfg = GeneralConfiguration()
        pkg_root = os.path.dirname(os.path.abspath(commonroad_crime.__file__))
        norm_icons = os.path.normpath(cfg.path_icons)
        self.assertEqual(norm_icons, os.path.join(pkg_root, "icons"))
        self.assertTrue(os.path.isdir(cfg.path_icons))

        crosswalk_path = os.path.join(cfg.path_icons, "crosswalk.png")
        self.assertTrue(os.path.isfile(crosswalk_path))
        img = plt.imread(crosswalk_path)
        self.assertGreaterEqual(img.ndim, 2)
        self.assertGreater(img.size, 0)

    def test_crosswalk_lookup_with_and_without_trailing_separator(self):
        cfg = GeneralConfiguration()
        with_sep = os.path.join(cfg.path_icons, "crosswalk.png")
        without_sep = os.path.join(cfg.path_icons.rstrip("/\\"), "crosswalk.png")
        self.assertTrue(os.path.isfile(with_sep))
        self.assertTrue(os.path.isfile(without_sep))

    def test_setup_py_declares_icons_package_data(self):
        repo_root = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
        setup_path = os.path.join(repo_root, "setup.py")
        with open(setup_path, "r", encoding="utf-8") as fh:
            tree = ast.parse(fh.read(), filename=setup_path)

        found_icons_glob = False
        for node in ast.walk(tree):
            if isinstance(node, ast.keyword) and node.arg == "package_data":
                data = ast.literal_eval(node.value)
                patterns = data.get("commonroad_crime", [])
                if "icons/*.png" in patterns:
                    found_icons_glob = True
        self.assertTrue(
            found_icons_glob,
            "setup.py must include 'icons/*.png' in package_data['commonroad_crime']",
        )


if __name__ == "__main__":
    unittest.main()
