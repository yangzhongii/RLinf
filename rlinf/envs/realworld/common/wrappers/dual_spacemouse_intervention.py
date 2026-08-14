# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Dual-arm SpaceMouse intervention wrapper for :class:`DualFrankaTCPEnv`.

Two SpaceMice (pyspacemouse device indices 0 and 1) each teleoperate one arm
of the dual-arm TCP env, whose action space is the absolute rot6d layout
``[L_xyz(3), L_rot6d(6), L_grip(1), R_xyz(3), R_rot6d(6), R_grip(1)]``.
``SpaceMouseExpert`` already maps its axes to the robot base frame
(``[-y, x, z, -roll, -pitch, -yaw]``), so each arm's 6D delta composes
directly against the current TCP pose returned by ``env.get_tcp_pose()``
without an adjoint transform. The left mouse button closes the gripper, the
right button opens it. Arms without recent input hold their current TCP pose
(the env only accepts absolute targets), so ``info["intervene_action"]`` is
always safe to record.
"""

from __future__ import annotations

import time
from typing import Optional

import gymnasium as gym
import numpy as np
from scipy.spatial.transform import Rotation as R

from rlinf.envs.realworld.common.spacemouse.spacemouse_expert import (
    SpaceMouseExpert,
)
from rlinf.envs.realworld.common.wrappers.spacemouse_intervention import (
    sample_gripper_action,
)
from rlinf.utils.rot6d import matrix_to_rot6d

_SIDES = ("left", "right")
_ARM_INDEX = {"left": 0, "right": 1}
_ACTION_DIM_PER_ARM = 10  # xyz(3) + rot6d(6) + gripper(1)


class DualSpacemouseIntervention(gym.ActionWrapper):
    """Two SpaceMice teleoperating ``DualFrankaTCPEnv-v1``.

    Modeled on :class:`DualFrankaTcpPicoIntervention` (per-arm delta-to-
    absolute composition against the measured TCP pose) and
    :class:`SpacemouseIntervention` (0.5 s intervention latch and mouse-button
    gripper toggling).
    """

    def __init__(
        self,
        env: gym.Env,
        gripper_enabled: bool = True,
        left_device_index: int = 0,
        right_device_index: int = 1,
        left_device_path: Optional[str] = None,
        right_device_path: Optional[str] = None,
        intervene_latch_s: float = 0.5,
    ):
        super().__init__(env)
        if getattr(env.unwrapped, "PER_ARM_ACTION_DIM", None) != _ACTION_DIM_PER_ARM:
            raise ValueError(
                "DualSpacemouseIntervention is implemented for "
                "DualFrankaTcpEnv-v1 only (PER_ARM_ACTION_DIM==10)."
            )

        self.gripper_enabled = gripper_enabled
        self.intervene_latch_s = intervene_latch_s
        # Path-based binding (udev by-path symlink) wins over the device
        # index; indices follow USB enumeration order and can swap between
        # reboots, paths are pinned to the physical USB port.
        self.experts = {
            "left": SpaceMouseExpert(
                device_index=left_device_index, device_path=left_device_path
            ),
            "right": SpaceMouseExpert(
                device_index=right_device_index, device_path=right_device_path
            ),
        }

        self._last_intervene = {"left": 0.0, "right": 0.0}
        self._active = {"left": False, "right": False}
        self._buttons = {"left": (False, False), "right": (False, False)}
        self._gripper_action = {"left": 0.95, "right": 0.95}
        if self.gripper_enabled:
            self._sync_gripper_action()

    def _sync_gripper_action(self) -> None:
        """Align each arm's cached gripper command with its env gripper state."""
        for side in _SIDES:
            state = self.get_wrapper_attr(f"_{side}_state")
            is_open = bool(getattr(state, "gripper_open", True))
            self._gripper_action[side] = float(sample_gripper_action(is_open)[0])

    @staticmethod
    def _tcp_pose_to_rot6d_action(
        tcp_pose: np.ndarray, gripper_action: float
    ) -> np.ndarray:
        rot6d = matrix_to_rot6d(R.from_quat(tcp_pose[3:7]).as_matrix())
        return np.concatenate(
            [
                np.asarray(tcp_pose[:3], dtype=np.float32),
                rot6d.astype(np.float32),
                np.array([gripper_action], dtype=np.float32),
            ],
            axis=0,
        )

    @staticmethod
    def _compose_expert_target(
        tcp_pose: np.ndarray, delta6: np.ndarray, action_scale: np.ndarray
    ) -> np.ndarray:
        """Compose a base-frame 6D delta against the current TCP pose."""
        current_pos = np.asarray(tcp_pose[:3], dtype=np.float64)
        current_rot = R.from_quat(np.asarray(tcp_pose[3:7], dtype=np.float64))

        target_pos = current_pos + delta6[:3] * float(action_scale[0])
        rot_action = np.asarray(delta6[3:6], dtype=np.float64)
        rot_norm = float(np.linalg.norm(rot_action))
        if rot_norm > 1.0:
            rot_action = rot_action / rot_norm
        target_rot = R.from_rotvec(rot_action * float(action_scale[1])) * current_rot
        return np.concatenate([target_pos, target_rot.as_quat()])

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        for side in _SIDES:
            self._last_intervene[side] = 0.0
            self._active[side] = False
            self._buttons[side] = (False, False)
        if self.gripper_enabled:
            self._sync_gripper_action()
        return obs, info

    def action(self, action: np.ndarray) -> tuple[np.ndarray, bool]:
        """Replace the policy action with per-arm SpaceMouse targets.

        Returns the (possibly) replaced action and whether any arm intervened
        in this step.
        """
        target_shape = self.env.action_space.shape
        if int(np.prod(target_shape)) != 2 * _ACTION_DIM_PER_ARM:
            raise ValueError(
                "DualSpacemouseIntervention expects DualFrankaTcpEnv's 20D "
                f"action space, got action_space.shape={target_shape}."
            )

        tcp_pose = np.asarray(self.get_wrapper_attr("get_tcp_pose")(), dtype=np.float64)
        if tcp_pose.size != 14:
            raise ValueError(
                "DualSpacemouseIntervention expects get_tcp_pose() to return "
                f"14 values, got shape {tcp_pose.shape}."
            )
        action_scale = self.get_wrapper_attr("get_action_scale")()

        # Base action: both arms hold their current TCP pose so untouched arms
        # never get driven toward the zero pose.
        new_action = np.concatenate(
            [
                self._tcp_pose_to_rot6d_action(
                    tcp_pose[_ARM_INDEX[side] * 7 : _ARM_INDEX[side] * 7 + 7],
                    self._gripper_action[side],
                )
                for side in _SIDES
            ]
        ).astype(np.float32)

        replaced_any = False
        for side in _SIDES:
            arm_idx = _ARM_INDEX[side]
            expert_a, buttons = self.experts[side].get_action()
            self._buttons[side] = tuple(bool(b) for b in buttons)

            any_input = np.linalg.norm(expert_a) > 0.001 or any(buttons)
            if any_input:
                self._last_intervene[side] = time.time()

            if self.gripper_enabled:
                if self._buttons[side][0]:  # left button closes gripper
                    self._gripper_action[side] = float(
                        sample_gripper_action(is_open=False)[0]
                    )
                    self._last_intervene[side] = time.time()
                elif self._buttons[side][1]:  # right button opens gripper
                    self._gripper_action[side] = float(
                        sample_gripper_action(is_open=True)[0]
                    )
                    self._last_intervene[side] = time.time()

            self._active[side] = (
                time.time() - self._last_intervene[side] < self.intervene_latch_s
            )
            if self._active[side]:
                target = self._compose_expert_target(
                    tcp_pose[arm_idx * 7 : arm_idx * 7 + 7], expert_a, action_scale
                )
                action_slice = slice(
                    arm_idx * _ACTION_DIM_PER_ARM,
                    arm_idx * _ACTION_DIM_PER_ARM + _ACTION_DIM_PER_ARM,
                )
                new_action[action_slice] = self._tcp_pose_to_rot6d_action(
                    target, self._gripper_action[side]
                )
                replaced_any = True

        new_action = np.clip(
            new_action,
            self.env.action_space.low.reshape(-1),
            self.env.action_space.high.reshape(-1),
        )
        return new_action.reshape(target_shape), replaced_any

    def step(self, action):
        new_action, replaced = self.action(action)

        obs, rew, done, truncated, info = self.env.step(new_action)
        # Always expose the executed (hold-based) action: the collect loop
        # feeds zeros, so without this untouched arms would record zero.
        info["intervene_action"] = new_action
        info["intervene_flag"] = np.ones(1) if replaced else np.zeros(1)
        info["left"] = self._active["left"]
        info["right"] = self._active["right"]
        for side in _SIDES:
            info[f"{side}_spacemouse_active"] = self._active[side]
            info[f"{side}_spacemouse_buttons"] = self._buttons[side]
        return obs, rew, done, truncated, info
