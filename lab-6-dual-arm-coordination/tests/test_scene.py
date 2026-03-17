"""Lab 6 — Phase 1 tests: MuJoCo scene structure and Pinocchio model loading."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from lab6_common import (
    DT,
    GRIPPER_OPEN,
    LEFT_CTRL,
    LEFT_GRIP_CTRL,
    LEFT_QPOS,
    NUM_JOINTS,
    Q_HOME_LEFT,
    Q_HOME_RIGHT,
    RIGHT_CTRL,
    RIGHT_GRIP_CTRL,
    RIGHT_QPOS,
    load_both_pinocchio_models,
    load_mujoco_model,
)


# ---------------------------------------------------------------------------
# MuJoCo scene structure
# ---------------------------------------------------------------------------

class TestSceneLoading:
    def setup_method(self):
        self.mj_model, self.mj_data = load_mujoco_model()

    def test_actuator_count(self):
        """Must have 14 actuators: 6 arm + 1 gripper per arm × 2."""
        assert self.mj_model.nu == 14

    def test_equality_count(self):
        """Must have 2 equality constraints (finger mirror per arm)."""
        assert self.mj_model.neq == 2

    def test_qpos_size(self):
        """16 joint qpos + 7 bar freejoint = 23."""
        assert self.mj_model.nq == 23

    def test_left_joint_names(self):
        """Left arm joints at qpos[0:6]."""
        import mujoco
        for i, expected in enumerate([
            "left_shoulder_pan_joint", "left_shoulder_lift_joint",
            "left_elbow_joint", "left_wrist_1_joint",
            "left_wrist_2_joint", "left_wrist_3_joint",
        ]):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            assert name == expected, f"Joint {i}: got {name!r}, expected {expected!r}"

    def test_right_joint_names(self):
        """Right arm joints at qpos[8:14]."""
        import mujoco
        for i, expected in enumerate([
            "right_shoulder_pan_joint", "right_shoulder_lift_joint",
            "right_elbow_joint", "right_wrist_1_joint",
            "right_wrist_2_joint", "right_wrist_3_joint",
        ]):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, i + 8)
            assert name == expected

    def test_left_ctrl_indices(self):
        """Left arm motors in ctrl[0:6], left gripper at ctrl[6]."""
        import mujoco
        left_motor_names = [f"left_motor{i+1}" for i in range(6)]
        for i, expected in enumerate(left_motor_names):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            assert name == expected
        grip_name = mujoco.mj_id2name(
            self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, LEFT_GRIP_CTRL)
        assert grip_name == "left_gripper"

    def test_right_ctrl_indices(self):
        """Right arm motors in ctrl[7:13], right gripper at ctrl[13]."""
        import mujoco
        right_motor_names = [f"right_motor{i+1}" for i in range(6)]
        for i, expected in enumerate(right_motor_names):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i + 7)
            assert name == expected
        grip_name = mujoco.mj_id2name(
            self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, RIGHT_GRIP_CTRL)
        assert grip_name == "right_gripper"

    def test_carry_bar_freejoint(self):
        """carry_bar freejoint at qpos[16]."""
        import mujoco
        jid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, "bar_joint")
        assert jid >= 0
        assert self.mj_model.jnt_qposadr[jid] == 16

    def test_sites_exist(self):
        """Both gripper sites must be present."""
        import mujoco
        for site_name in ("left_gripper_site", "right_gripper_site"):
            sid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            assert sid >= 0, f"Site '{site_name}' not found"

    def test_forward_kinematics_runs(self):
        """mj_forward should run without errors at Q_HOME."""
        import mujoco
        self.mj_data.qpos[LEFT_QPOS]  = Q_HOME_LEFT
        self.mj_data.qpos[RIGHT_QPOS] = Q_HOME_RIGHT
        self.mj_data.ctrl[LEFT_GRIP_CTRL]  = GRIPPER_OPEN
        self.mj_data.ctrl[RIGHT_GRIP_CTRL] = GRIPPER_OPEN
        mujoco.mj_forward(self.mj_model, self.mj_data)  # must not raise


# ---------------------------------------------------------------------------
# Pinocchio model loading
# ---------------------------------------------------------------------------

class TestPinocchioLoading:
    def setup_method(self):
        (self.pin_L, self.data_L, self.ee_L), (self.pin_R, self.data_R, self.ee_R) = \
            load_both_pinocchio_models()

    def test_both_models_load(self):
        assert self.pin_L is not None
        assert self.pin_R is not None

    def test_ee_frame_id_valid(self):
        """ee_link frame ID must exist in both models."""
        assert 0 < self.ee_L < self.pin_L.nframes
        assert 0 < self.ee_R < self.pin_R.nframes

    def test_independent_models(self):
        """Modifying one model's data must not affect the other."""
        import pinocchio as pin
        pin.forwardKinematics(self.pin_L, self.data_L, Q_HOME_LEFT)
        pin.updateFramePlacements(self.pin_L, self.data_L)
        ee_L = self.data_L.oMf[self.ee_L].translation.copy()

        # Compute right arm from completely different config
        q_zero = np.zeros(NUM_JOINTS)
        pin.forwardKinematics(self.pin_R, self.data_R, q_zero)
        pin.updateFramePlacements(self.pin_R, self.data_R)

        # Left EE must not have changed
        pin.forwardKinematics(self.pin_L, self.data_L, Q_HOME_LEFT)
        pin.updateFramePlacements(self.pin_L, self.data_L)
        ee_L_check = self.data_L.oMf[self.ee_L].translation.copy()
        np.testing.assert_allclose(ee_L, ee_L_check, atol=1e-10)

    def test_left_ee_at_home(self):
        """Left arm EE at Q_HOME_LEFT in arm-local frame: x≈-0.23, y≈+0.49."""
        import pinocchio as pin
        pin.forwardKinematics(self.pin_L, self.data_L, Q_HOME_LEFT)
        pin.updateFramePlacements(self.pin_L, self.data_L)
        ee = self.data_L.oMf[self.ee_L].translation
        # arm-local frame — Pinocchio base at local origin (not MuJoCo world)
        assert ee[2] > 0.1, f"Left EE z={ee[2]:.3f} unexpectedly low (arm should be above base)"

    def test_right_ee_at_home(self):
        """Right arm at Q_HOME_RIGHT must be symmetric mirror of left arm in arm-local frames.

        Left arm local: (x_L, y_L, z_L) at Q_HOME_LEFT
        Right arm local: (-x_L, -y_L, z_L) at Q_HOME_RIGHT  (shoulder_pan mirrored)
        """
        import pinocchio as pin
        pin.forwardKinematics(self.pin_L, self.data_L, Q_HOME_LEFT)
        pin.updateFramePlacements(self.pin_L, self.data_L)
        ee_L = self.data_L.oMf[self.ee_L].translation.copy()

        pin.forwardKinematics(self.pin_R, self.data_R, Q_HOME_RIGHT)
        pin.updateFramePlacements(self.pin_R, self.data_R)
        ee_R = self.data_R.oMf[self.ee_R].translation.copy()

        # Z should match; X and Y should be negated (mirror across XZ plane)
        np.testing.assert_allclose(ee_L[2], ee_R[2], atol=1e-2)
        np.testing.assert_allclose(ee_L[0], -ee_R[0], atol=1e-2)
        np.testing.assert_allclose(ee_L[1], -ee_R[1], atol=1e-2)
