"""Lab 6 — Bimanual pick-carry-place demonstration.

Runs the full cooperative carry cycle headless, then saves diagnostic plots to media/.

Usage:
    python3 bimanual_demo.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Add lab src to path
_SRC = Path(__file__).resolve().parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from lab6_common import (
    MEDIA_DIR,
    Q_HOME_LEFT,
    Q_HOME_RIGHT,
    load_both_pinocchio_models,
    load_mujoco_model,
)
from dual_arm_ik import compute_bimanual_configs, print_config_summary
from bimanual_state_machine import BimanualStateMachine


def run_demo() -> dict:
    """Load models, compute IK, and run the bimanual pick-carry-place cycle.

    Returns:
        Log dict from BimanualStateMachine.run().
    """
    print("=" * 60)
    print("Lab 6: Bimanual Pick-Carry-Place Demo")
    print("=" * 60)

    # --- Load models ---
    print("\n[1] Loading models...")
    mj_model, mj_data = load_mujoco_model()
    (pin_L, data_L, ee_L), (pin_R, data_R, ee_R) = load_both_pinocchio_models()
    print(f"    MuJoCo: nq={mj_model.nq}, nu={mj_model.nu}")
    print(f"    Pinocchio: {pin_L.nq} DOF per arm")

    # --- Compute IK configurations ---
    print("\n[2] Computing bimanual IK configurations...")
    cfgs = compute_bimanual_configs(pin_L, data_L, ee_L, pin_R, data_R, ee_R)
    print_config_summary(cfgs, pin_L, data_L, ee_L, pin_R, data_R, ee_R)

    # --- Run state machine ---
    print("\n[3] Running bimanual state machine...")
    sm = BimanualStateMachine(
        mj_model, mj_data,
        pin_L, data_L, ee_L,
        pin_R, data_R, ee_R,
        cfgs,
    )
    log = sm.run()

    print(f"\n    Total simulation time: {log['time'][-1]:.2f} s")
    print(f"    Total steps: {len(log['time'])}")

    # --- Plot results ---
    print("\n[4] Saving plots...")
    MEDIA_DIR.mkdir(parents=True, exist_ok=True)
    _plot_results(log)

    print(f"\n    Plots saved to: {MEDIA_DIR}/")
    return log


def _plot_results(log: dict) -> None:
    """Save diagnostic plots from the log.

    Args:
        log: Log dict from BimanualStateMachine.run().
    """
    t       = log["time"]
    ee_L    = log["ee_L"]
    ee_R    = log["ee_R"]
    bar_pos = log["bar_pos"]
    states  = log["state"]

    # State change boundaries
    state_changes = [0]
    for i in range(1, len(states)):
        if states[i] != states[i - 1]:
            state_changes.append(i)
    state_changes.append(len(states) - 1)

    # ----------------------------------------------------------------
    # Plot 1: EE trajectories (XYZ)
    # ----------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    labels = ["X (m)", "Y (m)", "Z (m)"]
    colors_L = ["#2196F3", "#1976D2", "#0D47A1"]
    colors_R = ["#F44336", "#D32F2F", "#B71C1C"]

    for ax, i, lbl in zip(axes, range(3), labels):
        ax.plot(t, ee_L[:, i], color=colors_L[i], lw=1.2, label="Left EE")
        ax.plot(t, ee_R[:, i], color=colors_R[i], lw=1.2, linestyle="--", label="Right EE")
        ax.set_ylabel(lbl, fontsize=11)
        ax.grid(True, alpha=0.3)
        for idx in state_changes[1:-1]:
            ax.axvline(t[idx], color="gray", lw=0.5, alpha=0.5)

    axes[0].legend(fontsize=10)
    axes[-1].set_xlabel("Time (s)", fontsize=11)
    fig.suptitle("Lab 6 — Bimanual EE Trajectories", fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(MEDIA_DIR / "ee_trajectories.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    # ----------------------------------------------------------------
    # Plot 2: Bar position
    # ----------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(12, 6), sharex=True)
    for ax, i, lbl in zip(axes, range(3), labels):
        ax.plot(t, bar_pos[:, i], color="#4CAF50", lw=1.5)
        ax.set_ylabel(lbl, fontsize=11)
        ax.grid(True, alpha=0.3)
        for idx in state_changes[1:-1]:
            ax.axvline(t[idx], color="gray", lw=0.5, alpha=0.5)

    axes[-1].set_xlabel("Time (s)", fontsize=11)
    fig.suptitle("Lab 6 — Carry Bar Position", fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(MEDIA_DIR / "bar_trajectory.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    # ----------------------------------------------------------------
    # Plot 3: Left vs right EE distance (should stay ~0.20 m apart in X)
    # ----------------------------------------------------------------
    ee_dist = np.linalg.norm(ee_L - ee_R, axis=1)
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, ee_dist * 1000, color="#9C27B0", lw=1.5, label="‖EE_L − EE_R‖")
    ax.axhline(200.0, color="gray", lw=1.0, linestyle="--", label="Nominal 200 mm")
    ax.set_xlabel("Time (s)", fontsize=11)
    ax.set_ylabel("EE Separation (mm)", fontsize=11)
    ax.set_title("Lab 6 — Bimanual EE Separation", fontsize=13, fontweight="bold")
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    for idx in state_changes[1:-1]:
        ax.axvline(t[idx], color="gray", lw=0.5, alpha=0.5)
    fig.tight_layout()
    fig.savefig(MEDIA_DIR / "ee_separation.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    # ----------------------------------------------------------------
    # Print state timing summary
    # ----------------------------------------------------------------
    print("\n  State timing:")
    prev_state = states[0]
    prev_t = t[0]
    for i in range(1, len(states)):
        if states[i] != prev_state:
            print(f"    {prev_state:20s}: {prev_t:.2f}s → {t[i-1]:.2f}s "
                  f"({t[i-1]-prev_t:.2f}s)")
            prev_state = states[i]
            prev_t = t[i]
    print(f"    {prev_state:20s}: {prev_t:.2f}s → {t[-1]:.2f}s "
          f"({t[-1]-prev_t:.2f}s)")

    bar_start = bar_pos[0]
    bar_end   = bar_pos[-1]
    print(f"\n  Bar displacement: {bar_start.round(3)} → {bar_end.round(3)}")
    print(f"  Bar moved: {np.linalg.norm(bar_end - bar_start)*1000:.1f} mm")


if __name__ == "__main__":
    log = run_demo()
    print("\nDone.")
