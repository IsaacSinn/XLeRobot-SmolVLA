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

"""
LeKiwi base (chassis) control logic (single source of truth, matches run_lekiwi_keyboard_only.py).
Shared by the keyboard teleop script and the Web UI "read commands from file" runner.
Contains no motor/keyboard/file I/O; only: key constants, smooth controller, three-wheel inverse
kinematics, and a minimal robot interface.
"""

import time

import numpy as np

# Directional keys and speed levels matching XLeRobot / run_lekiwi_keyboard_only
TELEOP_KEYS = {
    "forward": "i",
    "backward": "k",
    "left": "j",
    "right": "l",
    "rotate_left": "u",
    "rotate_right": "o",
    "speed_up": "n",
    "speed_down": "m",
    "quit": "b",
}
SPEED_LEVELS = [
    {"xy": 0.1, "theta": 30},
    {"xy": 0.2, "theta": 60},
    {"xy": 0.3, "theta": 90},
]

BASE_ACCELERATION_RATE = 10.0
BASE_DECELERATION_RATE = 10.0
BASE_MAX_SPEED = 6.0
MIN_VELOCITY_THRESHOLD = 0.02


class SmoothLeKiwiController:
    """Smooth controller matching run_lekiwi_keyboard_only / XLeRobot lekiwi_base_controller."""

    def __init__(self):
        self.current_speed = 0.0
        self.last_time = time.time()
        self.last_direction = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        self.is_moving = False

    def update(self, pressed_keys, robot):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        base_keys = [
            robot.teleop_keys["forward"],
            robot.teleop_keys["backward"],
            robot.teleop_keys["left"],
            robot.teleop_keys["right"],
            robot.teleop_keys["rotate_left"],
            robot.teleop_keys["rotate_right"],
        ]
        any_key_pressed = any(k in pressed_keys for k in base_keys)
        base_action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        if any_key_pressed:
            self.is_moving = True
            s = robot.speed_levels[robot.speed_index]
            if robot.teleop_keys["forward"] in pressed_keys:
                base_action["x.vel"] += s["xy"]
            if robot.teleop_keys["backward"] in pressed_keys:
                base_action["x.vel"] -= s["xy"]
            if robot.teleop_keys["left"] in pressed_keys:
                base_action["y.vel"] += s["xy"]
            if robot.teleop_keys["right"] in pressed_keys:
                base_action["y.vel"] -= s["xy"]
            if robot.teleop_keys["rotate_left"] in pressed_keys:
                base_action["theta.vel"] += s["theta"]
            if robot.teleop_keys["rotate_right"] in pressed_keys:
                base_action["theta.vel"] -= s["theta"]
            self.last_direction = base_action.copy()
            self.current_speed = min(
                self.current_speed + BASE_ACCELERATION_RATE * dt, BASE_MAX_SPEED
            )
        else:
            self.is_moving = False
            if self.current_speed > 0.01 and self.last_direction:
                base_action = self.last_direction.copy()
            self.current_speed = max(
                self.current_speed - BASE_DECELERATION_RATE * dt, 0.0
            )
        if base_action.get("x.vel") or base_action.get("y.vel") or base_action.get("theta.vel"):
            for key in base_action:
                if key.endswith(".vel"):
                    orig = base_action[key]
                    base_action[key] = orig * self.current_speed
                    if (
                        self.current_speed > 0.01
                        and abs(base_action[key]) < MIN_VELOCITY_THRESHOLD
                        and abs(orig) > 1e-6
                    ):
                        base_action[key] = (
                            MIN_VELOCITY_THRESHOLD if orig > 0 else -MIN_VELOCITY_THRESHOLD
                        )
        return base_action


def degps_to_raw(degps: float) -> int:
    steps_per_deg = 4096.0 / 360.0
    speed_int = int(round(degps * steps_per_deg))
    return max(-0x8000, min(0x7FFF, speed_int))


def body_to_wheel_raw(
    x: float,
    y: float,
    theta: float,
    wheel_radius: float = 0.05,
    base_radius: float = 0.125,
    max_raw: int = 3000,
) -> dict:
    theta_rad = theta * (np.pi / 180.0)
    vel = np.array([x, y, theta_rad])
    angles = np.radians(np.array([240, 0, 120]) - 90)
    m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])
    wheel_linear = m.dot(vel)
    wheel_radps = wheel_linear / wheel_radius
    wheel_degps = wheel_radps * (180.0 / np.pi)
    steps_per_deg = 4096.0 / 360.0
    raw_floats = [abs(d) * steps_per_deg for d in wheel_degps]
    max_raw_computed = max(raw_floats)
    if max_raw_computed > max_raw:
        scale = max_raw / max_raw_computed
        wheel_degps = wheel_degps * scale
    wheel_raw = [degps_to_raw(d) for d in wheel_degps]
    return {
        "base_left_wheel": wheel_raw[0],
        "base_back_wheel": wheel_raw[1],
        "base_right_wheel": wheel_raw[2],
    }


class MinimalLeKiwiRobot:
    """Minimal "robot" object for use by SmoothLeKiwiController; send_action requires bus.sync_write
    to be injected by the caller or implemented by a subclass."""

    teleop_keys = TELEOP_KEYS
    speed_levels = SPEED_LEVELS
    speed_index = 0

    def __init__(self, bus, port: str):
        self.bus = bus
        self.port = port

    def send_action(self, action: dict) -> None:
        x = action.get("x.vel", 0.0)
        y = action.get("y.vel", 0.0)
        theta = action.get("theta.vel", 0.0)
        wheel_cmds = body_to_wheel_raw(x, y, theta)
        self.bus.sync_write("Goal_Velocity", wheel_cmds)
