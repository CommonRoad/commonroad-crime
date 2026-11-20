"""Tests for cross-platform VRU crosswalk & shared-space criticality measures in CommonRoad-CriMe."""
from __future__ import annotations

import importlib
import math

from commonroad.scenario.obstacle import ObstacleType
import numpy as np
import pytest

from commonroad_crime.measure.vru import (
    build_vru_encounter_scenario,
    compute_crosswalk_pet,
    compute_vru_dst,
    compute_vru_tit,
    compute_vru_ttc,
    evaluate_vru_encounter_scenario,
)


def test_clean_importlib_and_package_imports_without_commonroad_dc() -> None:
    """Ensure `commonroad_crime.measure.vru` imports cleanly without C++ `commonroad_dc`."""
    mod = importlib.import_module("commonroad_crime.measure.vru.vru_conflict_measures")
    pkg = importlib.import_module("commonroad_crime.measure")
    assert hasattr(mod, "compute_vru_ttc")
    assert hasattr(pkg, "compute_vru_ttc")


def test_compute_vru_ttc_exact_cases() -> None:
    # Head-on / orthogonal collision in 2.0 s: distance 11.55 m - 1.55 m radius = 10.0 m @ 5 m/s
    ttc = compute_vru_ttc(
        pos_vru=[0.0, 0.0],
        vel_vru=[0.0, 0.0],
        pos_other=[11.55, 0.0],
        vel_other=[-5.0, 0.0],
        collision_distance=1.55,
    )
    assert ttc == pytest.approx(2.0, rel=1e-5)

    # Zero relative velocity (`a == 0`) -> inf
    assert math.isinf(
        compute_vru_ttc([0.0, 0.0], [1.2, 0.0], [5.0, 0.0], [1.2, 0.0], collision_distance=1.55)
    )

    # Diverging trajectories -> inf
    assert math.isinf(
        compute_vru_ttc([0.0, 0.0], [0.0, -1.2], [8.0, 0.0], [5.0, 0.0], collision_distance=1.55)
    )

    # Already overlapping -> 0.0
    assert compute_vru_ttc([0.0, 0.0], [0.0, 1.0], [0.8, 0.0], [-4.0, 0.0], collision_distance=1.55) == 0.0


def test_compute_crosswalk_pet_and_dst_and_tit() -> None:
    # Pedestrian crosses conflict box [-2, -2, 2, 2] along y from t=0..4s (exits y=+2 at t=3.0s)
    t_ped = np.linspace(0.0, 5.0, 51)
    xy_ped = np.column_stack([np.zeros_like(t_ped), -4.0 + 2.0 * t_ped])
    # Vehicle enters x=+2 at t=4.2s moving -10 m/s from x=44.0
    xy_veh = np.column_stack([44.0 - 10.0 * t_ped, np.zeros_like(t_ped)])

    pet = compute_crosswalk_pet(t_ped, xy_ped, t_ped, xy_veh, conflict_box=(-2.0, -2.0, 2.0, 2.0))
    assert pet == pytest.approx(1.2, abs=1e-3)

    # Simultaneous occupancy -> PET = 0.0
    xy_veh_hit = np.column_stack([22.0 - 10.0 * t_ped, np.zeros_like(t_ped)])
    assert compute_crosswalk_pet(t_ped, xy_ped, t_ped, xy_veh_hit, conflict_box=(-2.0, -2.0, 2.0, 2.0)) == 0.0

    # Deceleration to Safety Time (DST): urgent approach > calm distant approach
    dst_urgent = compute_vru_dst(
        pos_vru=[0.0, -1.0],
        vel_vru=[0.0, 1.2],
        pos_veh=[14.0, 0.0],
        vel_veh=[-11.0, 0.0],
    )
    dst_safe = compute_vru_dst(
        pos_vru=[0.0, -1.0],
        vel_vru=[0.0, 1.2],
        pos_veh=[55.0, 0.0],
        vel_veh=[-6.0, 0.0],
    )
    assert dst_urgent > 2.5
    assert dst_safe == pytest.approx(0.0)

    # Time-Integrated TTC (TIT)
    ttcs = [math.inf, 3.0, 2.0, 1.0, 0.5, 3.5]
    expected_tit = ((2.5 - 2.0) + (2.5 - 1.0) + (2.5 - 0.5)) * 0.1
    assert compute_vru_tit(ttcs, dt_s=0.1, tau_star_s=2.5) == pytest.approx(expected_tit)


def test_commonroad_scenario_builder_and_encounter_evaluation() -> None:
    dt_s = 0.10
    steps = np.arange(0, 50)
    ts = steps * dt_s

    # Near-miss encounter vs safe yielded encounter
    vru_xy = np.column_stack([np.zeros_like(ts), -3.5 + 1.4 * ts])
    vru_vel = np.column_stack([np.zeros_like(ts), np.full_like(ts, 1.4)])

    veh_xy_close = np.column_stack([26.0 - 9.5 * ts, np.zeros_like(ts)])
    veh_vel_close = np.column_stack([np.full_like(ts, -9.5), np.zeros_like(ts)])

    veh_xy_safe = np.column_stack([48.0 - 7.5 * ts, np.zeros_like(ts)])
    veh_vel_safe = np.column_stack([np.full_like(ts, -7.5), np.zeros_like(ts)])

    scen_close = build_vru_encounter_scenario(
        vru_xy, vru_vel, veh_xy_close, veh_vel_close, dt_s=dt_s, map_name="LEEDS_CLOSE"
    )
    scen_safe = build_vru_encounter_scenario(
        vru_xy,
        vru_vel,
        veh_xy_safe,
        veh_vel_safe,
        dt_s=dt_s,
        map_name="LEEDS_SAFE",
        vru_obstacle_type=ObstacleType.BICYCLE,
    )

    rep_close = evaluate_vru_encounter_scenario(scen_close)
    rep_safe = evaluate_vru_encounter_scenario(scen_safe)

    assert rep_close.min_ttc_s < rep_safe.min_ttc_s
    assert rep_close.pet_s < rep_safe.pet_s
    assert rep_close.max_dst_ms2 > rep_safe.max_dst_ms2
    assert rep_close.tit_s2 > rep_safe.tit_s2
