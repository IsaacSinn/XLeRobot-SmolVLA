#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

import logging
import time
from functools import cached_property
from itertools import chain
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_xlerobot import XLerobotConfig
from .lekiwi_base_controller import SmoothLeKiwiController

logger = logging.getLogger(__name__)


class XLerobot(Robot):
    """
    The robot includes a three omniwheel mobile base and a remote follower arm.
    The leader arm is connected locally (on the laptop) and its joint positions are recorded and then
    forwarded to the remote follower arm (after applying a safety clamp).
    In parallel, keyboard teleoperation is used to generate raw velocity commands for the wheels.
    """

    config_class = XLerobotConfig
    name = "xlerobot"

    def __init__(self, config: XLerobotConfig):
        super().__init__(config)
        self.config = config
        self.teleop_keys = config.teleop_keys
        self.speed_levels = [
            {"xy": 0.1, "theta": 30},  # slow
            {"xy": 0.2, "theta": 60},  # medium
            {"xy": 0.3, "theta": 90},  # fast
        ]
        self.speed_index = 0
        self._lekiwi_smooth_controller = SmoothLeKiwiController()

        # Base-only mode: all three wheel motors are on port1 (/dev/ttyACM0).
        # Default IDs are 7/8/9 (matching the original XLeRobot right-arm bus);
        # if the base motors are flashed separately as 1/2/3, change to Motor(1,...) Motor(2,...) Motor(3,...).
        cal_base = {k: v for k, v in self.calibration.items()
                    if k in ("base_left_wheel", "base_back_wheel", "base_right_wheel")}
        self.bus1 = FeetechMotorsBus(
            port=self.config.port1,
            motors={
                "base_left_wheel":  Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_back_wheel":  Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_right_wheel": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
            },
            calibration=cal_base if cal_base else self.calibration,
        )
        self.left_arm_motors = []
        self.right_arm_motors = []
        self.head_motors = []
        self.base_motors = [m for m in self.bus1.motors if m.startswith("base")]
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _state_ft(self) -> dict[str, type]:
        # Base-only mode: only base velocity state
        return dict.fromkeys(("x.vel", "y.vel", "theta.vel"), float)

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        return self.bus1.is_connected and all(
            cam.is_connected for cam in self.cameras.values()
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus1.connect()

        # Base wheels use velocity mode; no position calibration needed. Write default calibration directly.
        self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus1.is_calibrated

    def calibrate(self) -> None:
        """Base wheels use velocity mode; no position calibration needed.
        This automatically writes a full-range default calibration: homing_offset=0, range=0-4095.
        """
        logger.info("Base mode: writing default velocity calibration (no manual operation needed)")
        cal = {}
        for name, motor in self.bus1.motors.items():
            cal[name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=0,
                range_min=0,
                range_max=4095,
            )
        self.bus1.write_calibration(cal)
        self.calibration = cal
        self._save_calibration()
        logger.info("Base calibration complete")
        

    def configure(self):
        """Base mode: configure all three wheels for velocity control mode."""
        self.bus1.disable_torque()
        self.bus1.configure_motors()
        for name in self.base_motors:
            self.bus1.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        self.bus1.enable_torque()
        

    def setup_motors(self) -> None:
        """Base mode: only set the three wheel motor IDs."""
        for motor in reversed(self.base_motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus1.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus1.motors[motor].id}")
        

    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        steps_per_deg = 4096.0 / 360.0
        speed_in_steps = degps * steps_per_deg
        speed_int = int(round(speed_in_steps))
        # Cap the value to fit within signed 16-bit range (-32768 to 32767)
        if speed_int > 0x7FFF:
            speed_int = 0x7FFF  # 32767 -> maximum positive value
        elif speed_int < -0x8000:
            speed_int = -0x8000  # -32768 -> minimum negative value
        return speed_int

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        steps_per_deg = 4096.0 / 360.0
        magnitude = raw_speed
        degps = magnitude / steps_per_deg
        return degps


    def _get_wheel_kinematics_matrix(self, base_radius: float) -> "np.ndarray":
        """Build the three-omni-wheel kinematics matrix (standard omni-wheel formula).

        Physical layout (front of robot at φ=0°, CCW positive):
          φ=0°   : front wheel (lateral passive roller; does not rotate when moving forward, only
                   rotates when moving sideways)
          φ=120° : rear-left wheel (diagonal drive, ~30° off-rear)
          φ=240° : rear-right wheel (diagonal drive, ~30° off-rear)

        Standard formula: v_wheel_i = -sin(φ_i)*vx + cos(φ_i)*vy + R*ω
        Matrix per row: [-sin(φ_i), cos(φ_i), base_radius]

        Verification (forward vx=1, vy=0, ω=0):
          Front wheel (φ=0°)        : 0      → passive roller slide ✓
          Rear-left wheel (φ=120°)  : -√3/2  → rotates backward, provides forward force ✓
          Rear-right wheel (φ=240°) : +√3/2  → rotates forward, provides forward force ✓

        To change the wheel layout, just update wheel_angles_deg in XLerobotConfig.
        """
        angles = np.radians(np.array(self.config.wheel_angles_deg))
        return np.array([[-np.sin(a), np.cos(a), base_radius] for a in angles])

    def _body_to_wheel_raw(
        self,
        x: float,
        y: float,
        theta: float,
        max_raw: int = 3000,
    ) -> dict:
        """Convert a base body-frame velocity command into raw three-wheel speed commands.

        Args:
          x      : linear velocity in x direction (m/s)
          y      : linear velocity in y direction (m/s)
          theta  : angular velocity (deg/s)
          max_raw: max allowed raw command value per wheel; if exceeded, scale proportionally.

        Returns:
          {"base_left_wheel": int, "base_back_wheel": int, "base_right_wheel": int}
        """
        wheel_radius = self.config.wheel_radius
        base_radius = self.config.base_radius

        # theta: deg/s → rad/s
        theta_rad = theta * (np.pi / 180.0)
        velocity_vector = np.array([x, y, theta_rad])

        # Kinematics matrix: v_wheel = M · [vx, vy, omega_rad]
        m = self._get_wheel_kinematics_matrix(base_radius)
        wheel_linear_speeds = m.dot(velocity_vector)
        wheel_angular_speeds = wheel_linear_speeds / wheel_radius

        # rad/s → deg/s
        wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

        # If max_raw limit is exceeded, scale proportionally
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        max_raw_computed = max(raw_floats)
        if max_raw_computed > max_raw:
            wheel_degps = wheel_degps * (max_raw / max_raw_computed)

        wheel_raw = [self._degps_to_raw(deg) for deg in wheel_degps]

        return {
            "base_left_wheel": wheel_raw[0],
            "base_back_wheel": wheel_raw[1],
            "base_right_wheel": wheel_raw[2],
        }

    def _wheel_raw_to_body(
        self,
        left_wheel_speed,
        back_wheel_speed,
        right_wheel_speed,
    ) -> "dict[str, Any]":
        """Invert raw three-wheel speeds back into base body-frame velocity (inverse kinematics).

        Returns: {"x.vel": float, "y.vel": float, "theta.vel": float} (units m/s and deg/s)
        """
        wheel_radius = self.config.wheel_radius
        base_radius = self.config.base_radius

        wheel_degps = np.array([
            self._raw_to_degps(left_wheel_speed),
            self._raw_to_degps(back_wheel_speed),
            self._raw_to_degps(right_wheel_speed),
        ])

        # deg/s → rad/s → linear speed m/s
        wheel_linear_speeds = wheel_degps * (np.pi / 180.0) * wheel_radius

        # Inverse kinematics: [vx, vy, omega_rad] = M⁻¹ · v_wheel
        m = self._get_wheel_kinematics_matrix(base_radius)
        m_inv = np.linalg.inv(m)
        velocity_vector = m_inv.dot(wheel_linear_speeds)
        x, y, theta_rad = velocity_vector

        return {
            "x.vel": x,
            "y.vel": y,
            "theta.vel": theta_rad * (180.0 / np.pi),
        }

    def _from_keyboard_to_base_action(self, pressed_keys: np.ndarray):
        """LeKiwi three-wheel base: speed levels + smooth acceleration/deceleration; same logic
        shared by front-end and back-end."""
        # Speed level is updated by the n/m keys
        if self.teleop_keys["speed_up"] in pressed_keys:
            self.speed_index = min(self.speed_index + 1, 2)
        if self.teleop_keys["speed_down"] in pressed_keys:
            self.speed_index = max(self.speed_index - 1, 0)
        return self._lekiwi_smooth_controller.update(set(pressed_keys), self)

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        base_wheel_vel = self.bus1.sync_read("Present_Velocity", self.base_motors)
        base_vel = self._wheel_raw_to_body(
            base_wheel_vel["base_left_wheel"],
            base_wheel_vel["base_back_wheel"],
            base_wheel_vel["base_right_wheel"],
        )
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        camera_obs = self.get_camera_observation()
        return {**base_vel, **camera_obs}
    
    def get_camera_observation(self):
        obs_dict = {}
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
        
        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Base mode: only issue wheel velocity commands."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        base_goal_vel = {k: v for k, v in action.items() if k.endswith(".vel")}
        base_wheel_goal_vel = self._body_to_wheel_raw(
            base_goal_vel.get("x.vel", 0.0),
            base_goal_vel.get("y.vel", 0.0),
            base_goal_vel.get("theta.vel", 0.0),
        )
        if base_wheel_goal_vel:
            self.bus1.sync_write("Goal_Velocity", base_wheel_goal_vel)
        return base_goal_vel

    def stop_base(self):
        self.bus1.sync_write("Goal_Velocity", dict.fromkeys(self.base_motors, 0), num_retry=5)
        logger.info("Base motors stopped")

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_base()
        self.bus1.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
