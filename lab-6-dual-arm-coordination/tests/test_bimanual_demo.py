"""Lab 6 — Phase 3 smoke test: full bimanual pick-carry-place cycle.

This test runs the complete state machine headless and checks that:
  - The cycle completes without exceptions
  - The bar moved from its initial position (= it was carried)
  - The simulation time is positive and the log is well-formed
  - Both EE trajectories contain data
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from lab6_common import (
    BAR_PICK_POS,
    BAR_PLACE_POS,
    load_both_pinocchio_models,
    load_mujoco_model,
)
from dual_arm_ik import compute_bimanual_configs
from bimanual_state_machine import BimanualStateMachine, State


@pytest.fixture(scope="module")
def demo_log():
    """Run the full bimanual cycle and return the log dict."""
    mj_model, mj_data = load_mujoco_model()
    (pin_L, data_L, ee_L), (pin_R, data_R, ee_R) = load_both_pinocchio_models()
    cfgs = compute_bimanual_configs(pin_L, data_L, ee_L, pin_R, data_R, ee_R)
    sm = BimanualStateMachine(
        mj_model, mj_data,
        pin_L, data_L, ee_L,
        pin_R, data_R, ee_R,
        cfgs,
    )
    return sm.run()


class TestBimanualDemo:
    def test_log_keys(self, demo_log):
        """Log must contain all expected keys."""
        for key in ("time", "q_L", "q_R", "ee_L", "ee_R", "bar_pos", "state"):
            assert key in demo_log, f"Missing key: {key!r}"

    def test_simulation_runs(self, demo_log):
        """Simulation time must be positive and log non-empty."""
        assert demo_log["time"][-1] > 0.0
        assert len(demo_log["time"]) > 100

    def test_log_arrays_consistent(self, demo_log):
        """All array lengths must match."""
        N = len(demo_log["time"])
        for key in ("q_L", "q_R", "ee_L", "ee_R", "bar_pos"):
            assert demo_log[key].shape[0] == N, \
                f"{key} has {demo_log[key].shape[0]} rows, expected {N}"

    def test_both_states_visited(self, demo_log):
        """Key states must appear in the log."""
        visited = set(demo_log["state"])
        for state in ("PLAN_APPROACH", "EXEC_APPROACH", "DESCEND", "CLOSE",
                      "LIFT", "EXEC_TRANSPORT", "RELEASE", "DONE"):
            assert state in visited, f"State {state!r} was never visited"

    def test_bar_moved(self, demo_log):
        """Bar must have moved at least 50 mm from its initial position."""
        bar_start = demo_log["bar_pos"][0]
        bar_end   = demo_log["bar_pos"][-1]
        displacement_mm = np.linalg.norm(bar_end - bar_start) * 1000
        assert displacement_mm > 50.0, (
            f"Bar only moved {displacement_mm:.1f} mm — expected > 50 mm. "
            f"Start: {bar_start.round(3)}, End: {bar_end.round(3)}"
        )

    def test_ee_trajectories_nonzero(self, demo_log):
        """Both EE trajectories must have non-zero total path length."""
        ee_L = demo_log["ee_L"]
        ee_R = demo_log["ee_R"]
        path_len_L = np.sum(np.linalg.norm(np.diff(ee_L, axis=0), axis=1))
        path_len_R = np.sum(np.linalg.norm(np.diff(ee_R, axis=0), axis=1))
        assert path_len_L > 0.1, f"Left EE barely moved ({path_len_L:.3f} m total path)"
        assert path_len_R > 0.1, f"Right EE barely moved ({path_len_R:.3f} m total path)"

    def test_bar_z_during_lift(self, demo_log):
        """During LIFT state, bar Z must be above initial table height."""
        states = demo_log["state"]
        bar_z  = demo_log["bar_pos"][:, 2]
        lift_mask = np.array([s == "LIFT" for s in states])
        if lift_mask.any():
            max_z_during_lift = bar_z[lift_mask].max()
            assert max_z_during_lift > BAR_PICK_POS[2], (
                f"Bar Z during LIFT ({max_z_during_lift:.3f} m) "
                f"not above pick height ({BAR_PICK_POS[2]:.3f} m)"
            )
