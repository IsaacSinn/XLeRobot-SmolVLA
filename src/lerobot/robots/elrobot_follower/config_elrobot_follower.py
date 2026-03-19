#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig

# Joint limits derived from elrobot_follower.urdf (in degrees, for use_degrees=True mode).
# rev_motor_01~04, 06: ±1.5708 rad = ±90°
# rev_motor_05, 07:    ±2.9671 rad ≈ ±170° (near-full-turn wrist joints)
ELROBOT_JOINT_LIMITS_DEG: dict[str, tuple[float, float]] = {
    "joint_1": (-90.0, 90.0),
    "joint_2": (-90.0, 90.0),
    "joint_3": (-90.0, 90.0),
    "joint_4": (-90.0, 90.0),
    "joint_5": (-170.0, 170.0),
    "joint_6": (-90.0, 90.0),
    "joint_7": (-170.0, 170.0),
}


@dataclass
class ElrobotFollowerConfigBase:
    """Base configuration for Elrobot follower (joint_1..joint_7 + gripper, 8 motors)."""

    port: str

    disable_torque_on_disconnect: bool = True

    # Limit relative position step per cycle for safety. Scalar or dict per motor.
    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    use_degrees: bool = False

    # joint_5 and joint_7 are near-full-turn (±170°); joint_5 is the default.
    full_turn_motor: str = "joint_5"

    # Software joint limits applied in send_action. Only effective when use_degrees=True.
    # Defaults to URDF-derived limits. Set to None to disable.
    joint_limits: dict[str, tuple[float, float]] | None = field(
        default_factory=lambda: dict(ELROBOT_JOINT_LIMITS_DEG)
    )


@RobotConfig.register_subclass("elrobot_follower")
@dataclass
class ElrobotFollowerConfig(RobotConfig, ElrobotFollowerConfigBase):
    """Elrobot eight-motor Feetech follower config (joint_1..joint_7 + gripper)."""

    pass
