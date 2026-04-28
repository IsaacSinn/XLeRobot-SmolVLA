# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this code except in compliance with the License.
# See the License for the specific language governing permissions.

"""LeKiwi three-wheel omnidirectional base smooth controller, shared by front-end (client) and
back-end (robot)."""

import time

# LeKiwi three-wheel base smooth-control parameters (matches 4_xlerobot_2wheels)
BASE_ACCELERATION_RATE = 10.0   # Acceleration slope (speed/second)
BASE_DECELERATION_RATE = 10.0   # Deceleration slope
BASE_MAX_SPEED = 6.0            # Max speed multiplier
MIN_VELOCITY_THRESHOLD = 0.02   # Min velocity during deceleration; avoids abrupt motor stop


class SmoothLeKiwiController:
    """LeKiwi three-wheel omnidirectional base smooth controller: supports x/y/theta acceleration
    and deceleration."""

    def __init__(self):
        self.current_speed = 0.0
        self.last_time = time.time()
        self.last_direction = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        self.is_moving = False

    def update(self, pressed_keys, robot):
        """Update smoothed velocity based on pressed keys; returns body-frame velocity action
        (x.vel, y.vel, theta.vel).

        Args:
            pressed_keys: set of currently pressed keys (set or iterable; must support
                `k in pressed_keys`).
            robot: robot instance providing teleop_keys, speed_levels, and speed_index.

        Returns:
            dict: {"x.vel": float, "y.vel": float, "theta.vel": float}
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

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
            if not self.is_moving:
                self.is_moving = True
            speed_setting = robot.speed_levels[robot.speed_index]
            xy_speed = speed_setting["xy"]
            theta_speed = speed_setting["theta"]

            if robot.teleop_keys["forward"] in pressed_keys:
                base_action["x.vel"] += xy_speed
            if robot.teleop_keys["backward"] in pressed_keys:
                base_action["x.vel"] -= xy_speed
            if robot.teleop_keys["left"] in pressed_keys:
                base_action["y.vel"] += xy_speed
            if robot.teleop_keys["right"] in pressed_keys:
                base_action["y.vel"] -= xy_speed
            if robot.teleop_keys["rotate_left"] in pressed_keys:
                base_action["theta.vel"] += theta_speed
            if robot.teleop_keys["rotate_right"] in pressed_keys:
                base_action["theta.vel"] -= theta_speed

            self.last_direction = base_action.copy()
            self.current_speed += BASE_ACCELERATION_RATE * dt
            self.current_speed = min(self.current_speed, BASE_MAX_SPEED)
        else:
            if self.is_moving:
                self.is_moving = False
            if self.current_speed > 0.01 and self.last_direction:
                base_action = self.last_direction.copy()
            self.current_speed -= BASE_DECELERATION_RATE * dt
            self.current_speed = max(self.current_speed, 0.0)

        if base_action.get("x.vel") or base_action.get("y.vel") or base_action.get("theta.vel"):
            for key in base_action:
                if key.endswith(".vel"):
                    orig = base_action[key]
                    base_action[key] = orig * self.current_speed
                    if self.current_speed > 0.01 and abs(base_action[key]) < MIN_VELOCITY_THRESHOLD and abs(orig) > 1e-6:
                        base_action[key] = MIN_VELOCITY_THRESHOLD if orig > 0 else -MIN_VELOCITY_THRESHOLD

        return base_action
