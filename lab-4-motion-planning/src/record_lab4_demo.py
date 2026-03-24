"""Record the Lab 4 demo video: theory metrics + MuJoCo simulation.

Uses the shared video pipeline (tools/video_producer.py) with:
  Phase 1 — Animated metrics showing slalom planning results.
  Phase 2 — Native MuJoCo rendering of arm weaving through obstacles.
  Phase 3 — ffmpeg composition with title/end cards and crossfades.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl")

import mujoco
import numpy as np
import pinocchio as pin

_SRC = Path(__file__).resolve().parent
_LAB_DIR = _SRC.parent
_PROJECT_ROOT = _LAB_DIR.parent

if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
if str(_PROJECT_ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT / "tools"))

from capstone_demo import run_capstone
from lab4_common import (
    DT,
    MEDIA_DIR,
    MIN_CLEARANCE_M,
    NUM_JOINTS,
    OBSTACLES,
    SLALOM_LABELS,
    apply_arm_torques,
    clip_torques,
    get_ee_pos,
    get_mj_ee_pos,
    load_mujoco_model,
    load_pinocchio_model,
)
from video_producer import LabVideoProducer

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
LAB_NAME = "Lab 4: Motion Planning & Collision Avoidance"
OUTPUT_DIR = MEDIA_DIR


# ---------------------------------------------------------------------------
# Build metrics plots from capstone results
# ---------------------------------------------------------------------------

def build_plots(m: dict) -> list[dict]:
    """Build plot specs for the shared video producer."""
    result = m["result"]

    # Plot 1: EE trajectory XY with obstacles
    ee = result["ee_pos"]
    plot_ee = {
        "title": "End-Effector Slalom Path (XY)",
        "xlabel": "X (m)",
        "ylabel": "Y (m)",
        "type": "line",
        "series": [
            {"type": "line", "x": ee[:, 0], "y": ee[:, 1],
             "label": "EE Path", "color": "#58c4dd"},
        ],
    }

    # Plot 2: Obstacle clearance profile
    plot_clr = {
        "title": "Obstacle Clearance Profile",
        "xlabel": "Time (s)",
        "ylabel": "Min Clearance (m)",
        "data": (m["sample_times"], m["clr_profile"]),
        "color": "#9bde7e",
        "threshold": (MIN_CLEARANCE_M, f"{MIN_CLEARANCE_M} m threshold"),
    }

    # Plot 3: Tracking error
    err = np.max(np.abs(result["q_actual"] - result["q_desired"]), axis=1)
    plot_err = {
        "title": "Max Joint Tracking Error",
        "xlabel": "Time (s)",
        "ylabel": "Error (rad)",
        "data": (result["time"], err),
        "color": "#c792ea",
    }

    # Plot 4: Joint torques
    tau_max = np.max(np.abs(result["tau"]), axis=1)
    plot_tau = {
        "title": "Max Joint Torque",
        "xlabel": "Time (s)",
        "ylabel": "Torque (Nm)",
        "data": (result["time"], tau_max),
        "color": "#ff9e66",
    }

    return [plot_ee, plot_clr, plot_err, plot_tau]


def build_kpi(m: dict) -> dict[str, str]:
    """Build KPI overlay for the metrics clip."""
    return {
        "Waypoints": f"{len(SLALOM_LABELS)}",
        "Tree Nodes": f"{m['tree_nodes']}",
        "Min Clearance": f"{m['min_clearance']:.3f} m",
        "Track RMS": f"{m['rms']:.4f} rad",
        "Final Error": f"{m['final_err']:.4f} rad",
        "Duration": f"{m['times'][-1]:.2f} s",
    }


# ---------------------------------------------------------------------------
# Controller for MuJoCo simulation recording
# ---------------------------------------------------------------------------

def make_trajectory_controller(
    t_traj: np.ndarray,
    q_traj: np.ndarray,
    qd_traj: np.ndarray,
    Kp: float = 400.0,
    Kd: float = 40.0,
) -> tuple:
    """Create a controller function for MuJoCo recording."""
    pin_model, pin_data, _ = load_pinocchio_model()
    traj_duration = float(t_traj[-1])
    total_duration = traj_duration + 1.0

    def controller_fn(model: mujoco.MjModel, data: mujoco.MjData, step: int) -> bool | None:
        t = step * DT

        if t <= traj_duration:
            q_d = np.array([np.interp(t, t_traj, q_traj[:, j]) for j in range(NUM_JOINTS)])
            qd_d = np.array([np.interp(t, t_traj, qd_traj[:, j]) for j in range(NUM_JOINTS)])
        else:
            q_d = q_traj[-1]
            qd_d = np.zeros(NUM_JOINTS)

        q = data.qpos[:NUM_JOINTS].copy()
        qd = data.qvel[:NUM_JOINTS].copy()

        pin.computeGeneralizedGravity(pin_model, pin_data, q)
        g = pin_data.g.copy()

        tau = Kp * (q_d - q) + Kd * (qd_d - qd) + g
        tau = clip_torques(tau)
        apply_arm_torques(model, data, tau)

        if t > total_duration:
            return False
        return None

    return controller_fn, total_duration


def make_status_text_fn(m: dict):
    """Create a status overlay for the simulation recording."""
    traj_duration = float(m["times"][-1])

    def status_fn(model, data, step, current_time):
        ee_pos = get_mj_ee_pos(model, data)
        phase = "Weaving" if current_time <= traj_duration else "Holding"
        return {
            "Slalom Execution": "",
            f"Time: {current_time:.1f}s / {traj_duration:.1f}s": "",
            f"Phase: {phase}": "",
            f"EE: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]": "",
        }

    return status_fn


def camera_schedule(current_time: float, progress: float) -> dict:
    """Top-down start transitioning to orbit view."""
    if progress < 0.40:
        return {
            "lookat": [0.55, 0.0, 0.42],
            "distance": 1.40,
            "elevation": -85.0,
            "azimuth": 90.0,
        }
    blend = min(1.0, (progress - 0.40) / 0.60)
    eased = 0.5 - 0.5 * np.cos(np.pi * blend)
    return {
        "lookat": [0.55 + 0.03 * eased, 0.0, 0.42],
        "distance": 1.40 + 0.12 * eased,
        "elevation": -85.0 + 49.0 * eased,
        "azimuth": 90.0 + 38.0 * eased,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> Path:
    print("=" * 60)
    print("Lab 4 Demo Video — Slalom Planning + Simulation")
    print("=" * 60)

    # Step 1: Run the capstone pipeline
    print("\n[1/3] Running slalom planning and execution pipeline...")
    metrics = run_capstone()

    # Step 2: Build video producer
    producer = LabVideoProducer(lab_name=LAB_NAME, output_dir=OUTPUT_DIR)

    # Step 3: Create metrics clip
    print("\n[2/3] Creating animated metrics clip...")
    plots = build_plots(metrics)
    kpi = build_kpi(metrics)
    metrics_clip = producer.create_metrics_clip(
        plots=plots,
        kpi_overlay=kpi,
        title_text="Slalom Planning Through Obstacles",
        duration_sec=12.0,
    )
    print(f"  Metrics clip: {metrics_clip}")

    # Step 4: Record MuJoCo simulation
    print("\n[3/3] Recording MuJoCo simulation...")
    mj_model, mj_data = load_mujoco_model()

    mj_data.qpos[:NUM_JOINTS] = metrics["configs"][0].copy()
    mj_data.qvel[:] = 0.0
    mujoco.mj_forward(mj_model, mj_data)

    controller_fn, total_duration = make_trajectory_controller(
        metrics["times"],
        metrics["q_traj"],
        metrics["qd_traj"],
    )

    # Planned EE path for visual trace
    pin_model, pin_data, ee_fid = load_pinocchio_model()
    planned_ee = np.array([
        get_ee_pos(pin_model, pin_data, ee_fid, q)
        for q in metrics["dense_path"]
    ])

    simulation_clip = producer.record_simulation(
        model=mj_model,
        data=mj_data,
        controller_fn=controller_fn,
        camera_name="orbit_45",
        playback_speed=0.3,
        trace_ee=True,
        trace_site_name="2f85_pinch",
        duration_sec=total_duration + 1.0,
        camera_schedule=camera_schedule,
        planned_trace_points=planned_ee,
        planned_trace_color=(0.22, 0.92, 0.35, 0.80),
        actual_trace_color=(0.22, 0.52, 0.98, 0.90),
        status_text_fn=make_status_text_fn(metrics),
        start_hold_sec=1.5,
        end_hold_sec=2.5,
    )
    print(f"  Simulation clip: {simulation_clip}")

    # Step 5: Compose final video
    print("\nComposing final video...")
    final_path = producer.compose_final_video(
        metrics_clip=metrics_clip,
        simulation_clip=simulation_clip,
        title_card_text="Lab 4: Motion Planning & Collision Avoidance",
        crossfade_sec=0.6,
    )
    print(f"\nDemo video saved: {final_path}")
    return final_path


if __name__ == "__main__":
    main()
