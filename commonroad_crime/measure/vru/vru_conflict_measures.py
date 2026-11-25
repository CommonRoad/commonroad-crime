"""Cross-platform VRU crosswalk and shared-space criticality measures."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from commonroad.geometry.shape import Circle, Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.scenario.state import CustomState, InitialState
from commonroad.scenario.trajectory import Trajectory
import numpy as np


@dataclass(frozen=True)
class VruEncounterCriticalityReport:
    """Criticality profile for a CommonRoad VRU-vehicle or VRU-cyclist encounter."""

    scenario_id: str
    min_ttc_s: float
    pet_s: float
    max_dst_ms2: float
    tit_s2: float
    ttc_series_s: tuple[float, ...]


def compute_vru_ttc(
    pos_vru: Sequence[float] | np.ndarray,
    vel_vru: Sequence[float] | np.ndarray,
    pos_other: Sequence[float] | np.ndarray,
    vel_other: Sequence[float] | np.ndarray,
    collision_distance: float = 1.55,
) -> float:
    """
    Compute exact continuous-space 2D Time-to-Collision (`TTC`, seconds) between a VRU
    (pedestrian, wheelchair user, child, cyclist) and an approaching road user.

    Handles zero relative velocity (`a == 0`), diverging paths, and immediate overlap
    (`0.0`) cleanly without requiring compiled `commonroad_dc` C++ extensions.
    """
    p_a = np.asarray(pos_vru, dtype=float)[:2]
    v_a = np.asarray(vel_vru, dtype=float)[:2]
    p_b = np.asarray(pos_other, dtype=float)[:2]
    v_b = np.asarray(vel_other, dtype=float)[:2]

    rel_p = p_b - p_a
    r_coll = max(0.05, float(collision_distance))
    dist_now = float(np.linalg.norm(rel_p))
    if dist_now <= r_coll:
        return 0.0

    rel_v = v_b - v_a
    a = float(np.dot(rel_v, rel_v))
    if a < 1e-9:
        return math.inf

    b = 2.0 * float(np.dot(rel_p, rel_v))
    if b >= 0.0:
        return math.inf

    c = float(np.dot(rel_p, rel_p)) - r_coll * r_coll
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return math.inf

    sqrt_d = math.sqrt(disc)
    t1 = (-b - sqrt_d) / (2.0 * a)
    t2 = (-b + sqrt_d) / (2.0 * a)
    if t1 <= 0.0 <= t2:
        return 0.0
    pos_roots = [t for t in (t1, t2) if t > 0.0]
    return min(pos_roots) if pos_roots else math.inf


def _segment_box_entry_exit_times(
    times_s: np.ndarray,
    traj_xy: np.ndarray,
    conflict_box: tuple[float, float, float, float],
) -> tuple[float, float] | None:
    """Compute exact interpolated `[t_entry, t_exit]` across `conflict_box`."""
    xmin, ymin, xmax, ymax = (float(v) for v in conflict_box)
    if len(times_s) == 0 or len(traj_xy) == 0:
        return None

    t_in: float | None = None
    t_out: float | None = None

    def _inside(pt: np.ndarray) -> bool:
        return xmin <= float(pt[0]) <= xmax and ymin <= float(pt[1]) <= ymax

    for k in range(len(times_s)):
        if _inside(traj_xy[k]):
            tk = float(times_s[k])
            t_in = tk if t_in is None else min(t_in, tk)
            t_out = tk if t_out is None else max(t_out, tk)

    # Liang-Barsky parametric segment clipping for sub-step accuracy
    for k in range(len(times_s) - 1):
        p0 = traj_xy[k]
        p1 = traj_xy[k + 1]
        dp = p1 - p0
        u1, u2 = 0.0, 1.0
        p_arr = [-dp[0], dp[0], -dp[1], dp[1]]
        q_arr = [p0[0] - xmin, xmax - p0[0], p0[1] - ymin, ymax - p0[1]]
        valid = True
        for pi, qi in zip(p_arr, q_arr):
            if abs(pi) < 1e-12:
                if qi < 0.0:
                    valid = False
                    break
            else:
                r = qi / pi
                if pi < 0.0:
                    if r > u2:
                        valid = False
                        break
                    if r > u1:
                        u1 = r
                else:
                    if r < u1:
                        valid = False
                        break
                    if r < u2:
                        u2 = r
        if valid and u1 <= u2:
            t0 = float(times_s[k])
            dt = float(times_s[k + 1] - times_s[k])
            seg_in = t0 + u1 * dt
            seg_out = t0 + u2 * dt
            t_in = seg_in if t_in is None else min(t_in, seg_in)
            t_out = seg_out if t_out is None else max(t_out, seg_out)

    if t_in is None or t_out is None:
        return None
    return t_in, t_out


def compute_crosswalk_pet(
    vru_times_s: Sequence[float] | np.ndarray,
    vru_xy: Sequence[Sequence[float]] | np.ndarray,
    veh_times_s: Sequence[float] | np.ndarray,
    veh_xy: Sequence[Sequence[float]] | np.ndarray,
    conflict_box: tuple[float, float, float, float] = (-3.5, -3.5, 3.5, 3.5),
) -> float:
    """
    Compute exact continuous-time Post-Encroachment Time (`PET`, seconds) over a
    crosswalk or shared-space conflict zone `(xmin, ymin, xmax, ymax)`.

    Returns:
        - `0.0` if VRU and vehicle simultaneously occupy `conflict_box`
        - `t_second_entry - t_first_exit` (> 0) for sequential passages
        - `math.inf` if either road user never enters `conflict_box`.
    """
    t_vru = np.asarray(vru_times_s, dtype=float)
    xy_vru = np.asarray(vru_xy, dtype=float)
    t_veh = np.asarray(veh_times_s, dtype=float)
    xy_veh = np.asarray(veh_xy, dtype=float)

    win_vru = _segment_box_entry_exit_times(t_vru, xy_vru, conflict_box)
    win_veh = _segment_box_entry_exit_times(t_veh, xy_veh, conflict_box)
    if win_vru is None or win_veh is None:
        return math.inf

    in_a, out_a = win_vru
    in_b, out_b = win_veh
    if out_a < in_b:
        return float(in_b - out_a)
    if out_b < in_a:
        return float(in_a - out_b)
    return 0.0


def compute_vru_dst(
    pos_vru: Sequence[float] | np.ndarray,
    vel_vru: Sequence[float] | np.ndarray,
    pos_veh: Sequence[float] | np.ndarray,
    vel_veh: Sequence[float] | np.ndarray,
    conflict_point: Sequence[float] | np.ndarray = (0.0, 0.0),
    clear_radius_m: float = 2.2,
) -> float:
    """
    Compute Deceleration to Safety Time (`DST`, m/s^2, Hupfer 1997) for a vehicle
    approaching a VRU crossing or occupying `conflict_point`.

    Measures the constant longitudinal deceleration required for the vehicle to arrive
    at the conflict boundary no earlier than the VRU exits `clear_radius_m`:
        a_DST = max(0, 2 * (v_veh * t_vru_clear - d_veh) / t_vru_clear^2)
    """
    p_vru = np.asarray(pos_vru, dtype=float)[:2]
    v_vru = np.asarray(vel_vru, dtype=float)[:2]
    p_veh = np.asarray(pos_veh, dtype=float)[:2]
    v_veh = np.asarray(vel_veh, dtype=float)[:2]
    cp = np.asarray(conflict_point, dtype=float)[:2]

    to_cp_veh = cp - p_veh
    dist_cp_veh = float(np.linalg.norm(to_cp_veh))
    if dist_cp_veh < 1e-6:
        return 0.0
    unit_to_cp = to_cp_veh / dist_cp_veh
    closing_speed_veh = float(np.dot(v_veh, unit_to_cp))
    if closing_speed_veh <= 0.05:
        return 0.0

    d_veh_eff = max(0.25, dist_cp_veh - float(clear_radius_m))
    vru_speed = float(np.linalg.norm(v_vru))
    dist_vru_cp = float(np.linalg.norm(p_vru - cp))

    if vru_speed < 0.15:
        if dist_vru_cp <= float(clear_radius_m):
            # Stationary VRU inside crosswalk -> required stopping deceleration
            return float((closing_speed_veh**2) / (2.0 * d_veh_eff))
        return 0.0

    # Remaining distance until VRU clears past conflict zone
    unit_vru = v_vru / vru_speed
    proj_along = float(np.dot(cp - p_vru, unit_vru))
    rem_clear_dist = proj_along + float(clear_radius_m)
    if rem_clear_dist <= 0.0:
        return 0.0

    t_clear = rem_clear_dist / vru_speed
    denom = max(t_clear * t_clear, 1e-4)
    a_dst = 2.0 * (closing_speed_veh * t_clear - d_veh_eff) / denom
    return float(max(0.0, a_dst))


def compute_vru_tit(
    ttc_series_s: Sequence[float] | np.ndarray,
    dt_s: float = 0.05,
    tau_star_s: float = 2.5,
) -> float:
    """
    Compute Time-Integrated Time-to-Collision (`TIT`, s^2, Minderhoud & Bovy 2001):
        TIT = sum_t max(0, tau_star - TTC(t)) * dt
    """
    arr = np.asarray(ttc_series_s, dtype=float)
    if arr.size == 0:
        return 0.0
    valid = np.where(np.isfinite(arr), arr, math.inf)
    deficit = np.maximum(0.0, float(tau_star_s) - valid)
    return float(np.sum(deficit) * float(dt_s))


def _build_dynamic_obstacle(
    obstacle_id: int,
    obstacle_type: ObstacleType,
    shape: Circle | Rectangle,
    xy_traj: np.ndarray,
    vel_traj: np.ndarray,
) -> DynamicObstacle:
    v0 = vel_traj[0]
    speed0 = float(np.linalg.norm(v0))
    yaw0 = math.atan2(float(v0[1]), float(v0[0])) if speed0 > 1e-6 else 0.0
    init_state = InitialState(
        time_step=0,
        position=xy_traj[0].astype(float),
        orientation=yaw0,
        velocity=speed0,
        acceleration=0.0,
        yaw_rate=0.0,
    )
    states = []
    for k in range(1, len(xy_traj)):
        vk = vel_traj[k]
        sk = float(np.linalg.norm(vk))
        yk = math.atan2(float(vk[1]), float(vk[0])) if sk > 1e-6 else yaw0
        states.append(
            CustomState(
                time_step=k,
                position=xy_traj[k].astype(float),
                orientation=yk,
                velocity=sk,
            )
        )
    traj = Trajectory(initial_time_step=1, state_list=states)
    pred = TrajectoryPrediction(trajectory=traj, shape=shape)
    return DynamicObstacle(
        obstacle_id=obstacle_id,
        obstacle_type=obstacle_type,
        obstacle_shape=shape,
        initial_state=init_state,
        prediction=pred,
    )


def build_vru_encounter_scenario(
    vru_xy: Sequence[Sequence[float]] | np.ndarray,
    vru_vel: Sequence[Sequence[float]] | np.ndarray,
    veh_xy: Sequence[Sequence[float]] | np.ndarray,
    veh_vel: Sequence[Sequence[float]] | np.ndarray,
    dt_s: float = 0.05,
    map_name: str = "VRU_CROSSING",
    vru_obstacle_type: ObstacleType = ObstacleType.PEDESTRIAN,
    vru_id: int = 101,
    veh_id: int = 201,
) -> Scenario:
    """
    Construct a native `commonroad.scenario.scenario.Scenario` containing a VRU
    (`ObstacleType.PEDESTRIAN` or `ObstacleType.BICYCLE`) and a vehicle (`CAR`).
    """
    xy_p = np.asarray(vru_xy, dtype=float)
    v_p = np.asarray(vru_vel, dtype=float)
    xy_v = np.asarray(veh_xy, dtype=float)
    v_v = np.asarray(veh_vel, dtype=float)
    if len(xy_p) < 2 or len(xy_v) < 2 or len(xy_p) != len(xy_v):
        raise ValueError("vru_xy and veh_xy must have equal length >= 2")

    scen = Scenario(dt=float(dt_s), scenario_id=ScenarioID(map_name=map_name[:16]))
    vru_obs = _build_dynamic_obstacle(
        obstacle_id=vru_id,
        obstacle_type=vru_obstacle_type,
        shape=Circle(radius=0.30),
        xy_traj=xy_p,
        vel_traj=v_p,
    )
    veh_obs = _build_dynamic_obstacle(
        obstacle_id=veh_id,
        obstacle_type=ObstacleType.CAR,
        shape=Rectangle(length=4.60, width=1.85),
        xy_traj=xy_v,
        vel_traj=v_v,
    )
    scen.add_objects([vru_obs, veh_obs])
    return scen


def evaluate_vru_encounter_scenario(
    scenario: Scenario,
    vru_id: int = 101,
    veh_id: int = 201,
    collision_distance: float = 1.55,
    conflict_box: tuple[float, float, float, float] = (-3.5, -3.5, 3.5, 3.5),
    tau_star_s: float = 2.5,
) -> VruEncounterCriticalityReport:
    """Evaluate `TTC`, `PET`, `DST`, and `TIT` directly on a CommonRoad `Scenario`."""
    vru_obs = scenario.obstacle_by_id(vru_id)
    veh_obs = scenario.obstacle_by_id(veh_id)
    if vru_obs is None or veh_obs is None:
        raise KeyError(f"Obstacles ({vru_id}, {veh_id}) not found in Scenario")

    dt_s = float(scenario.dt)
    max_step = vru_obs.prediction.final_time_step
    times_s: list[float] = []
    p_xys: list[np.ndarray] = []
    v_xys: list[np.ndarray] = []
    ttc_list: list[float] = []
    dst_list: list[float] = []

    conflict_center = (
        0.5 * (conflict_box[0] + conflict_box[2]),
        0.5 * (conflict_box[1] + conflict_box[3]),
    )

    for step in range(0, max_step + 1):
        sp = vru_obs.state_at_time(step)
        sv = veh_obs.state_at_time(step)
        if sp is None or sv is None:
            continue
        vp_vec = np.array(
            [
                float(sp.velocity) * math.cos(float(sp.orientation)),
                float(sp.velocity) * math.sin(float(sp.orientation)),
            ],
            dtype=float,
        )
        vv_vec = np.array(
            [
                float(sv.velocity) * math.cos(float(sv.orientation)),
                float(sv.velocity) * math.sin(float(sv.orientation)),
            ],
            dtype=float,
        )
        pos_p = np.asarray(sp.position, dtype=float)
        pos_v = np.asarray(sv.position, dtype=float)

        times_s.append(step * dt_s)
        p_xys.append(pos_p)
        v_xys.append(pos_v)
        ttc_list.append(
            compute_vru_ttc(
                pos_p, vp_vec, pos_v, vv_vec, collision_distance=collision_distance
            )
        )
        dst_list.append(
            compute_vru_dst(
                pos_p, vp_vec, pos_v, vv_vec, conflict_point=conflict_center
            )
        )

    pet_val = compute_crosswalk_pet(
        times_s, p_xys, times_s, v_xys, conflict_box=conflict_box
    )
    tit_val = compute_vru_tit(ttc_list, dt_s=dt_s, tau_star_s=tau_star_s)

    return VruEncounterCriticalityReport(
        scenario_id=str(scenario.scenario_id),
        min_ttc_s=min(ttc_list) if ttc_list else math.inf,
        pet_s=pet_val,
        max_dst_ms2=max(dst_list) if dst_list else 0.0,
        tit_s2=tit_val,
        ttc_series_s=tuple(ttc_list),
    )
