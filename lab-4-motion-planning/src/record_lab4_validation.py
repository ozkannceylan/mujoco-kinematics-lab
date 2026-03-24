"""Record a Lab 4 validation video using native MuJoCo offscreen rendering.

Demonstrates the UR5e weaving through tabletop obstacle boxes using the
slalom planning pipeline (IK -> per-segment RRT* -> TOPP-RA -> execute).

Usage:
    python3 lab-4-motion-planning/src/record_lab4_validation.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import imageio.v3 as iio
import mujoco
import numpy as np
import pinocchio as pin

_SRC = Path(__file__).resolve().parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from capstone_demo import ClearanceAwareChecker, plan_segments, solve_waypoints
from collision_checker import CollisionChecker
from lab4_common import (
    ACC_LIMITS,
    ACC_SCALE,
    DT,
    MEDIA_DIR,
    NUM_JOINTS,
    OBSTACLES,
    SLALOM_WAYPOINTS,
    VEL_LIMITS,
    VEL_SCALE,
    apply_arm_torques,
    clip_torques,
    densify_path,
    get_ee_pos,
    get_mj_ee_pos,
    load_mujoco_model,
    load_pinocchio_model,
)
from trajectory_smoother import parameterize_topp_ra

# ── Video settings ────────────────────────────────────────────────────────
VIDEO_PATH = MEDIA_DIR / "lab4_validation_real_stack.mp4"
WIDTH, HEIGHT = 1280, 720
FPS = 30

# Slower limits for smooth, readable video motion
VIDEO_VEL_SCALE = 0.12
VIDEO_ACC_SCALE = 0.10


# ── Scene overlay helpers ─────────────────────────────────────────────────

def _add_sphere_to_scene(
    scn: mujoco.MjvScene,
    pos: np.ndarray,
    size: float,
    rgba: tuple[float, float, float, float],
) -> None:
    """Append a visual-only sphere geom to the MuJoCo scene."""
    if scn.ngeom >= scn.maxgeom:
        return
    mujoco.mjv_initGeom(
        scn.geoms[scn.ngeom],
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=np.array([size, 0.0, 0.0]),
        pos=np.asarray(pos, dtype=np.float64),
        mat=np.eye(3).flatten(),
        rgba=np.asarray(rgba, dtype=np.float32),
    )
    scn.ngeom += 1


def _add_trail_to_scene(
    scn: mujoco.MjvScene,
    trail: np.ndarray,
    size: float = 0.005,
    rgba: tuple[float, float, float, float] = (0.15, 0.50, 0.95, 0.90),
) -> None:
    """Draw the EE trail as blue spheres."""
    for pos in trail:
        _add_sphere_to_scene(scn, pos, size, rgba)


def _add_markers(
    scn: mujoco.MjvScene,
    start_pos: np.ndarray,
    goal_pos: np.ndarray,
) -> None:
    """Add green (start) and red (goal) marker spheres."""
    _add_sphere_to_scene(scn, start_pos, 0.018, (0.1, 0.75, 0.2, 1.0))
    _add_sphere_to_scene(scn, goal_pos, 0.018, (0.9, 0.1, 0.1, 1.0))


# ── Main recording function ──────────────────────────────────────────────

def record_video(output_path: Path = VIDEO_PATH) -> Path:
    """Run the slalom pipeline and record the execution to MP4."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 72)
    print("Lab 4 Validation — Slalom Obstacle Avoidance")
    print("=" * 72)

    # ── 1. Solve IK + plan ─────────────────────────────────────────────────
    print("\n  [1/5] Solving IK and planning...")
    cc = CollisionChecker(obstacle_specs=list(OBSTACLES))
    cc_clr = ClearanceAwareChecker(cc)
    pin_model, pin_data, ee_fid = load_pinocchio_model()

    configs, clearances = solve_waypoints(cc)
    full_path, planning_time, _ = plan_segments(cc_clr, configs)

    print(f"  Waypoints: {len(SLALOM_WAYPOINTS)}, path: {len(full_path)} wps")
    print(f"  Planning time: {planning_time:.2f}s")

    for i, (cfg, clr) in enumerate(zip(configs, clearances)):
        ee = get_ee_pos(pin_model, pin_data, ee_fid, cfg)
        print(f"    WP{i}: EE=[{ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f}] clr={clr:.3f}m")

    ee_start = get_ee_pos(pin_model, pin_data, ee_fid, configs[0])
    ee_goal = get_ee_pos(pin_model, pin_data, ee_fid, configs[-1])

    # ── 2. Time-parameterize ───────────────────────────────────────────────
    print("\n  [2/5] Time-parameterizing...")
    dense = densify_path(full_path, max_step=0.02)
    times, q_traj, qd_traj, _ = parameterize_topp_ra(
        dense,
        VIDEO_VEL_SCALE * VEL_LIMITS,
        VIDEO_ACC_SCALE * ACC_LIMITS,
        dt=DT,
    )
    traj_duration = float(times[-1])
    print(f"  Duration: {traj_duration:.2f}s, samples: {len(times)}")

    # ── 3. Execute in MuJoCo ──────────────────────────────────────────────
    print("\n  [3/5] Executing in MuJoCo...")
    mj_model, mj_data = load_mujoco_model()
    pin_ctrl_model, pin_ctrl_data, _ = load_pinocchio_model()

    mj_data.qpos[:NUM_JOINTS] = q_traj[0]
    mj_data.qvel[:NUM_JOINTS] = 0.0
    mujoco.mj_forward(mj_model, mj_data)

    settle_time = 0.5
    total_duration = traj_duration + settle_time
    n_steps = int(total_duration / DT) + 1

    q_log = np.zeros((n_steps, NUM_JOINTS))
    ee_log = np.zeros((n_steps, 3))

    for step in range(n_steps):
        t = step * DT
        if t <= traj_duration:
            q_d = np.array([np.interp(t, times, q_traj[:, j]) for j in range(NUM_JOINTS)])
            qd_d = np.array([np.interp(t, times, qd_traj[:, j]) for j in range(NUM_JOINTS)])
        else:
            q_d = q_traj[-1]
            qd_d = np.zeros(NUM_JOINTS)

        q = mj_data.qpos[:NUM_JOINTS].copy()
        qd = mj_data.qvel[:NUM_JOINTS].copy()

        pin.computeGeneralizedGravity(pin_ctrl_model, pin_ctrl_data, q)
        g = pin_ctrl_data.g.copy()
        tau = 400.0 * (q_d - q) + 40.0 * (qd_d - qd) + g
        tau = clip_torques(tau)
        apply_arm_torques(mj_model, mj_data, tau)

        q_log[step] = q
        ee_log[step] = get_mj_ee_pos(mj_model, mj_data)
        mujoco.mj_step(mj_model, mj_data)

    q_desired = np.array([
        [np.interp(np.clip(s * DT, 0, traj_duration), times, q_traj[:, j])
         for j in range(NUM_JOINTS)]
        for s in range(n_steps)
    ])
    rms = float(np.sqrt(np.mean((q_log - q_desired) ** 2)))
    print(f"  Execution complete: {n_steps} steps, RMS={rms:.4f} rad")

    # ── 4. Render frames ──────────────────────────────────────────────────
    print(f"\n  [4/5] Rendering {WIDTH}x{HEIGHT} @ {FPS} fps...")
    mj_data.qpos[:NUM_JOINTS] = q_log[0]
    mj_data.qvel[:] = 0.0
    mujoco.mj_forward(mj_model, mj_data)

    renderer = mujoco.Renderer(mj_model, HEIGHT, WIDTH)
    cam = mujoco.MjvCamera()
    cam.lookat = np.array([0.55, 0.0, 0.42])
    cam.distance = 1.50

    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False

    # Camera: start top-down, transition to orbit
    sim_dt = DT
    stride = max(1, int(round(1.0 / (FPS * sim_dt))))
    frame_indices = np.arange(0, n_steps, stride, dtype=int)
    if frame_indices[-1] != n_steps - 1:
        frame_indices = np.append(frame_indices, n_steps - 1)

    HOLD_START = FPS * 2
    HOLD_END = FPS * 3
    total_motion = len(frame_indices)
    frames: list[np.ndarray] = []

    # Start hold — top-down view
    cam.elevation = -85.0
    cam.azimuth = 90.0
    renderer.update_scene(mj_data, camera=cam, scene_option=opt)
    _add_markers(renderer.scene, ee_start, ee_goal)
    start_frame = renderer.render().copy()
    frames.extend([start_frame] * HOLD_START)

    # Motion frames — transition from top-down to orbit
    for frame_i, idx in enumerate(frame_indices):
        mj_data.qpos[:NUM_JOINTS] = q_log[idx]
        mj_data.qvel[:] = 0.0
        mujoco.mj_forward(mj_model, mj_data)

        progress = frame_i / max(total_motion - 1, 1)
        if progress < 0.45:
            cam.elevation = -85.0
            cam.azimuth = 90.0
        else:
            blend = min(1.0, (progress - 0.45) / 0.55)
            eased = 0.5 - 0.5 * np.cos(np.pi * blend)
            cam.elevation = -85.0 + eased * 49.0  # -> -36
            cam.azimuth = 90.0 + eased * 38.0     # -> 128

        renderer.update_scene(mj_data, camera=cam, scene_option=opt)

        trail_pts = ee_log[:idx + 1:max(1, stride * 2)]
        _add_trail_to_scene(renderer.scene, trail_pts)
        _add_sphere_to_scene(
            renderer.scene, ee_log[idx], 0.012, (0.9, 0.1, 0.1, 1.0),
        )
        _add_markers(renderer.scene, ee_start, ee_goal)

        frames.append(renderer.render().copy())

        if frame_i % 50 == 0:
            pct = 100.0 * frame_i / total_motion
            print(f"\r      {pct:5.1f}%", end="", flush=True)

    print(f"\r      100.0%  ({total_motion} frames)")

    # End hold
    mj_data.qpos[:NUM_JOINTS] = q_log[-1]
    mj_data.qvel[:] = 0.0
    mujoco.mj_forward(mj_model, mj_data)
    cam.elevation = -36.0
    cam.azimuth = 128.0
    renderer.update_scene(mj_data, camera=cam, scene_option=opt)
    _add_trail_to_scene(renderer.scene, ee_log[::max(1, stride * 2)])
    _add_markers(renderer.scene, ee_start, ee_goal)
    end_frame = renderer.render().copy()
    frames.extend([end_frame] * HOLD_END)

    renderer.close()

    # ── 5. Write video ────────────────────────────────────────────────────
    print(f"\n  [5/5] Writing {len(frames)} frames ({len(frames)/FPS:.1f}s)...")
    iio.imwrite(str(output_path), np.stack(frames), fps=FPS, codec="libx264")

    print(f"\n  Video saved: {output_path}")
    print(f"  Duration: {len(frames)/FPS:.1f} s")
    print(f"  Waypoints: {len(SLALOM_WAYPOINTS)}, path: {len(full_path)} wps")
    print("\n" + "=" * 72)
    print("Lab 4 Validation — COMPLETE")
    print("=" * 72)

    return output_path


def main() -> None:
    """Entry point."""
    record_video()


if __name__ == "__main__":
    main()
