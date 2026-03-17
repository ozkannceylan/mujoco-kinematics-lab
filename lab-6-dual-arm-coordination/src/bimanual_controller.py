"""Lab 6 — Bimanual impedance controller.

Runs both arms in a single MuJoCo step with:
  tau = Kp*(q_d - q) + Kd*(qd_d - qd) + g(q)

Each arm's gravity vector is computed independently via Pinocchio.
Torques are written to mj_data.ctrl simultaneously before mj_step().

Also provides a Cartesian impedance step for the descend/lift phases,
importing compute_impedance_torque from Lab 3.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

from lab6_common import (
    LEFT_CTRL,
    NUM_JOINTS,
    RIGHT_CTRL,
    clip_torques,
    add_lab_src_to_path,
)

# Cross-lab: Lab 3 impedance controller
add_lab_src_to_path("lab-3-dynamics-force-control")
from b1_impedance_controller import ImpedanceGains, compute_impedance_torque  # noqa: E402


# ---------------------------------------------------------------------------
# Bimanual controller
# ---------------------------------------------------------------------------

class BimanualController:
    """Joint-space impedance controller for two UR5e arms.

    Both arms are updated in one call and written to mj_data.ctrl before
    the caller invokes mj_step().  Gravity compensation uses Pinocchio.

    Args:
        pin_L: Pinocchio model for left arm.
        data_L: Pinocchio data for left arm.
        ee_fid_L: Left arm EE frame ID.
        pin_R: Pinocchio model for right arm.
        data_R: Pinocchio data for right arm.
        ee_fid_R: Right arm EE frame ID.
        Kp: Joint-space proportional gain.
        Kd: Joint-space derivative gain.
        Kp_cart: Cartesian translational stiffness (N/m).
        Kd_cart: Cartesian translational damping (N·s/m).
    """

    def __init__(
        self,
        pin_L,
        data_L,
        ee_fid_L: int,
        pin_R,
        data_R,
        ee_fid_R: int,
        Kp: float = 400.0,
        Kd: float = 40.0,
        Kp_cart: float = 600.0,
        Kd_cart: float = 60.0,
    ) -> None:
        self.pin_L    = pin_L
        self.data_L   = data_L
        self.ee_fid_L = ee_fid_L
        self.pin_R    = pin_R
        self.data_R   = data_R
        self.ee_fid_R = ee_fid_R

        self.Kp = Kp
        self.Kd = Kd
        self.cart_gains = ImpedanceGains(
            K_p=np.diag([Kp_cart] * 3),
            K_d=np.diag([Kd_cart] * 3),
        )

    # ------------------------------------------------------------------
    # Joint-space impedance
    # ------------------------------------------------------------------

    def compute_joint_torques(
        self,
        pin_model,
        pin_data,
        q: np.ndarray,
        qd: np.ndarray,
        q_d: np.ndarray,
        qd_d: np.ndarray,
    ) -> np.ndarray:
        """Compute joint-impedance torques with gravity compensation.

        tau = Kp*(q_d - q) + Kd*(qd_d - qd) + g(q)

        Args:
            pin_model: Pinocchio model.
            pin_data: Pinocchio data.
            q: Current joint positions (6,).
            qd: Current joint velocities (6,).
            q_d: Desired joint positions (6,).
            qd_d: Desired joint velocities (6,).

        Returns:
            Clipped joint torques (6,).
        """
        pin.computeGeneralizedGravity(pin_model, pin_data, q)
        g = pin_data.g.copy()
        tau = (self.Kp * (q_d - q)
               + self.Kd * (qd_d - qd)
               + g)
        return clip_torques(tau)

    def apply_joint_impedance(
        self,
        mj_data,
        q_d_L: np.ndarray,
        qd_d_L: np.ndarray,
        q_d_R: np.ndarray,
        qd_d_R: np.ndarray,
    ) -> None:
        """Write joint-space impedance torques for both arms into mj_data.ctrl.

        Gravity compensation is computed independently for each arm.

        Args:
            mj_data: MuJoCo data (mutated: ctrl is written).
            q_d_L: Desired left arm joint positions (6,).
            qd_d_L: Desired left arm joint velocities (6,).
            q_d_R: Desired right arm joint positions (6,).
            qd_d_R: Desired right arm joint velocities (6,).
        """
        from lab6_common import LEFT_QPOS, RIGHT_QPOS
        q_L  = mj_data.qpos[LEFT_QPOS].copy()
        qd_L = mj_data.qvel[LEFT_QPOS].copy()
        q_R  = mj_data.qpos[RIGHT_QPOS].copy()
        qd_R = mj_data.qvel[RIGHT_QPOS].copy()

        tau_L = self.compute_joint_torques(
            self.pin_L, self.data_L, q_L, qd_L, q_d_L, qd_d_L)
        tau_R = self.compute_joint_torques(
            self.pin_R, self.data_R, q_R, qd_R, q_d_R, qd_d_R)

        mj_data.ctrl[LEFT_CTRL]  = tau_L
        mj_data.ctrl[RIGHT_CTRL] = tau_R

    # ------------------------------------------------------------------
    # Cartesian impedance (descend / lift)
    # ------------------------------------------------------------------

    def apply_cartesian_impedance(
        self,
        mj_data,
        x_des_L: np.ndarray,
        R_des_L: np.ndarray,
        x_des_R: np.ndarray,
        R_des_R: np.ndarray,
    ) -> None:
        """Write Cartesian impedance torques for both arms into mj_data.ctrl.

        tau = J^T * (K_p * Δx + K_d * Δẋ) + g(q)

        Args:
            mj_data: MuJoCo data (mutated: ctrl is written).
            x_des_L: Desired left arm EE position (3,).
            R_des_L: Desired left arm EE rotation (3×3).
            x_des_R: Desired right arm EE position (3,).
            R_des_R: Desired right arm EE rotation (3×3).
        """
        from lab6_common import LEFT_QPOS, RIGHT_QPOS
        q_L  = mj_data.qpos[LEFT_QPOS].copy()
        qd_L = mj_data.qvel[LEFT_QPOS].copy()
        q_R  = mj_data.qpos[RIGHT_QPOS].copy()
        qd_R = mj_data.qvel[RIGHT_QPOS].copy()

        tau_L = compute_impedance_torque(
            self.pin_L, self.data_L, self.ee_fid_L,
            q_L, qd_L,
            x_des_L, R_des_L, None,
            self.cart_gains,
        )
        tau_R = compute_impedance_torque(
            self.pin_R, self.data_R, self.ee_fid_R,
            q_R, qd_R,
            x_des_R, R_des_R, None,
            self.cart_gains,
        )

        mj_data.ctrl[LEFT_CTRL]  = clip_torques(tau_L)
        mj_data.ctrl[RIGHT_CTRL] = clip_torques(tau_R)

    # ------------------------------------------------------------------
    # Gravity hold (gripper action, waiting)
    # ------------------------------------------------------------------

    def apply_gravity_hold(self, mj_data, q_hold_L: np.ndarray, q_hold_R: np.ndarray) -> None:
        """Hold both arms at fixed configs with joint impedance (zero velocity desired).

        Args:
            mj_data: MuJoCo data (mutated).
            q_hold_L: Left arm hold configuration (6,).
            q_hold_R: Right arm hold configuration (6,).
        """
        self.apply_joint_impedance(
            mj_data,
            q_hold_L, np.zeros(NUM_JOINTS),
            q_hold_R, np.zeros(NUM_JOINTS),
        )
