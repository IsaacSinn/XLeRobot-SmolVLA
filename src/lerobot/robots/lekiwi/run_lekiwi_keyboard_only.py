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

"""
LeKiwi base-only keyboard teleoperation (same keys as 4_xlerobot_teleop_keyboard).
- Keys: i forward, k back, j left, l right, u rotate-left, o rotate-right, n speed-up, m speed-down
- Connects directly to servos 7/8/9 on /dev/ttyACM0; no ZMQ and no arm required
- Requires chmod or adding to dialout group first: sudo chmod 666 /dev/ttyACM0

Base control logic comes from this package's lekiwi_base_control.py (shared with Web UI base).

Run (from project root or XLeRobot/software):
  PYTHONPATH=src python -m lerobot.robots.lekiwi.run_lekiwi_keyboard_only
  PYTHONPATH=src python -m lerobot.robots.lekiwi.run_lekiwi_keyboard_only --port /dev/ttyACM0
Or after installation:
  lerobot-lekiwi-keyboard
"""

import argparse
import sys
import time

# Base control logic: single source from this package (shared with the Web UI "read commands from file" runner)
from lerobot.robots.lekiwi.lekiwi_base_control import (
    MinimalLeKiwiRobot,
    SmoothLeKiwiController,
    TELEOP_KEYS,
    SPEED_LEVELS,
)

# Motors and keyboard: prefer this repo's lerobot.motors; fall back to lerobot_new (legacy compat)
try:
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
except ImportError:
    sys.path.insert(0, "/home/lakesenberg/lerobot_new/src")
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop
from lerobot.teleoperators.keyboard.configuration_keyboard import KeyboardTeleopConfig


def main():
    parser = argparse.ArgumentParser(
        description="LeKiwi base keyboard teleoperation (same keys as 4_xlerobot_teleop_keyboard)"
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Servo serial port")
    args = parser.parse_args()

    print(f"[LeKiwi] Connecting to {args.port} (motors 7/8/9)...")
    bus = FeetechMotorsBus(
        port=args.port,
        motors={
            "base_left_wheel": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_back_wheel": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_right_wheel": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
        },
    )
    bus.connect()
    for name in ["base_left_wheel", "base_back_wheel", "base_right_wheel"]:
        bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
    bus.enable_torque()
    print("[LeKiwi] Base ready (VELOCITY mode). Keys: i/k fwd/back j/l left/right u/o rotate n/m speed b quit")

    robot = MinimalLeKiwiRobot(bus, args.port)
    smooth = SmoothLeKiwiController()
    keyboard = KeyboardTeleop(KeyboardTeleopConfig())
    keyboard.connect()

    try:
        while True:
            pressed = set(keyboard.get_action().keys())
            if "b" in pressed or "B" in pressed:
                print("[LeKiwi] Quit")
                break
            if "n" in pressed or "N" in pressed:
                robot.speed_index = min(robot.speed_index + 1, 2)
            if "m" in pressed or "M" in pressed:
                robot.speed_index = max(robot.speed_index - 1, 0)
            base_action = smooth.update(pressed, robot)
            robot.send_action(base_action)
            time.sleep(1.0 / 50)
    finally:
        bus.sync_write(
            "Goal_Velocity",
            {"base_left_wheel": 0, "base_back_wheel": 0, "base_right_wheel": 0},
        )
        bus.disconnect()
        keyboard.disconnect()
        print("[LeKiwi] Stopped and disconnected.")


if __name__ == "__main__":
    main()
