"""MuJoCo simulation wrapper for the UR5e.

Provides a clean API for:
  - Loading the scene
  - Setting joint positions / velocities
  - Applying torque controls
  - Stepping the simulation
  - Reading end-effector pose
  - Rendering frames (offscreen)
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

from ur5e_common import MJCF_SCENE_PATH, NUM_JOINTS


@dataclass
class RobotState:
    """Snapshot of the robot's state."""
    qpos: np.ndarray       # joint positions (6,)
    qvel: np.ndarray       # joint velocities (6,)
    ee_pos: np.ndarray     # end-effector position (3,)
    ee_rot: np.ndarray     # end-effector rotation matrix (3,3)
    time: float


class UR5eSimulator:
    """Wrapper around MuJoCo for UR5e simulation.

    Usage:
        sim = UR5eSimulator()
        sim.set_qpos(q_home)
        for _ in range(1000):
            sim.set_ctrl(torques)
            sim.step()
        state = sim.get_state()
    """

    def __init__(self, scene_path: Optional[Path] = None) -> None:
        """Initialize the simulator.

        Args:
            scene_path: Path to MJCF scene. Defaults to the lab scene.
        """
        path = scene_path or MJCF_SCENE_PATH
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)

        # Cache site ID for end-effector
        self.ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site"
        )

        # Joint indices (first NUM_JOINTS joints belong to the robot)
        self._nq_robot = NUM_JOINTS

        # Initialize to home keyframe if available
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)

    @property
    def dt(self) -> float:
        """Simulation timestep (seconds)."""
        return self.model.opt.timestep

    @property
    def time(self) -> float:
        """Current simulation time (seconds)."""
        return self.data.time

    def set_qpos(self, q: np.ndarray) -> None:
        """Set robot joint positions and run forward kinematics.

        Args:
            q: Joint positions (6,).
        """
        self.data.qpos[:self._nq_robot] = q
        mujoco.mj_forward(self.model, self.data)

    def set_qvel(self, qvel: np.ndarray) -> None:
        """Set robot joint velocities.

        Args:
            qvel: Joint velocities (6,).
        """
        self.data.qvel[:self._nq_robot] = qvel

    def set_ctrl(self, ctrl: np.ndarray) -> None:
        """Set actuator controls (torques or position targets depending on model).

        Args:
            ctrl: Control signals (6,).
        """
        self.data.ctrl[:self._nq_robot] = ctrl

    def step(self, n: int = 1) -> None:
        """Step the simulation forward.

        Args:
            n: Number of steps to take.
        """
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def get_state(self) -> RobotState:
        """Get the current robot state."""
        return RobotState(
            qpos=self.data.qpos[:self._nq_robot].copy(),
            qvel=self.data.qvel[:self._nq_robot].copy(),
            ee_pos=self.data.site_xpos[self.ee_site_id].copy(),
            ee_rot=self.data.site_xmat[self.ee_site_id].reshape(3, 3).copy(),
            time=self.data.time,
        )

    def get_ee_pos(self) -> np.ndarray:
        """Get end-effector position (3,)."""
        return self.data.site_xpos[self.ee_site_id].copy()

    def get_ee_rot(self) -> np.ndarray:
        """Get end-effector rotation matrix (3,3)."""
        return self.data.site_xmat[self.ee_site_id].reshape(3, 3).copy()

    def get_qfrc_bias(self) -> np.ndarray:
        """Get bias forces (gravity + Coriolis) for comparison with Pinocchio."""
        return self.data.qfrc_bias[:self._nq_robot].copy()

    def reset(self, keyframe: int = 0) -> None:
        """Reset simulation to a keyframe.

        Args:
            keyframe: Keyframe index (0 = home).
        """
        if keyframe < self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, keyframe)
        else:
            mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def render_frame(self, width: int = 640, height: int = 480,
                     camera: Optional[str] = None) -> np.ndarray:
        """Render an offscreen frame.

        Args:
            width: Image width in pixels.
            height: Image height in pixels.
            camera: Camera name (None for free camera).

        Returns:
            RGB image array (height, width, 3).
        """
        renderer = mujoco.Renderer(self.model, height, width)
        renderer.update_scene(self.data, camera=camera)
        img = renderer.render()
        renderer.close()
        return img
