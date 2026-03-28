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
飞特 STS/SO 舵机自动校准时使用的默认配置与常量。

供 auto_calibration.py、lerobot_measure_feetech_ranges.py 等统一引用。
"""

from .. import Motor, MotorNormMode

# ---------------------------------------------------------------------------
# 默认位置范围（range_min, range_max）
# ---------------------------------------------------------------------------
# SO/STS 常见关节名到 (range_min, range_max) 的默认映射（raw 编码值，含 4096 分辨率）
# 若关节名不在表中，将使用全量程 (0, max_res)
SO_STS_DEFAULT_RANGES: dict[str, tuple[int, int]] = {
    "shoulder_pan": (0, 4095),
    "shoulder_lift": (0, 4095),
    "elbow_flex": (0, 4095),
    "wrist_flex": (0, 4095),
    "wrist_roll": (0, 4095),
    "gripper": (0, 4095),
}



# SO 六轴臂关节名 -> 舵机编号（用于打印等）
SO_MOTOR_NUMBERS: dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

# SO 六轴臂关节名列表（与 SO_MOTOR_NUMBERS 顺序一致）
MOTOR_NAMES: list[str] = list(SO_MOTOR_NUMBERS.keys())

# SO 标定用电机表（name -> Motor，归一化模式为标定/脚本用）
SO_FOLLOWER_MOTORS: dict[str, Motor] = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}


def motor_label(name: str) -> str:
    """用于打印的舵机标签：name(id)，例如 shoulder_pan(1)。"""
    n = SO_MOTOR_NUMBERS.get(name, "")
    return f"{name}({n})" if n != "" else name


# ---------------------------------------------------------------------------
# 分辨率与中位（4096 步/圈）
# ---------------------------------------------------------------------------
FULL_TURN = 4096
MID_POS = 2047
STS_HALF_TURN_RAW = 2047  # 与 MID_POS 相同，回中/归一化时用

# Homing_Offset 寄存器为 12 位符号-数值编码 (sign_bit_index=11)，可表示 [-2047, 2047]
HOMING_OFFSET_MAX_MAG = 2047

# ---------------------------------------------------------------------------
# 校准/测量参数
# ---------------------------------------------------------------------------
DEFAULT_VELOCITY_LIMIT = 1000       # 校准探测限位速度（恒速模式 Goal_Velocity）
DEFAULT_MAX_TORQUE = 1000           # 最大扭矩（Max_Torque_Limit）
DEFAULT_TORQUE_LIMIT = 380         # 扭矩限制（Torque_Limit）
DEFAULT_ACCELERATION = 50           # 加速度（与项目 configure_motors 一致）
DEFAULT_POS_SPEED = 1000            # 伺服模式 WritePosEx 默认速度
DEFAULT_P_COEFFICIENT = 16          # PID P 系数（与 so_follower 一致）
DEFAULT_I_COEFFICIENT = 0           # PID I 系数
DEFAULT_D_COEFFICIENT = 32          # PID D 系数
DEFAULT_TIMEOUT = 20.0              # 校准单方向限位等待超时（秒）
POSITION_TOLERANCE = 20             # 到达目标位置的容差（步）
# 限位判断 AND 条件：速度近零 + 位置稳定 + Moving=0（优先于 Status BIT5）
STALL_VELOCITY_THRESHOLD = 3        # 速度接近 0 的阈值（|Present_Velocity| 小于此视为停）
STALL_POSITION_DELTA_THRESHOLD = 3  # 相邻采样位置变化小于此步数视为不动
OVERLOAD_SETTLE_TIME = 0.2          # 堵转后关扭矩等待恢复时间（秒）
SAFE_IO_RETRIES = 5                 # 安全读写重试次数
SAFE_IO_INTERVAL = 0.2              # 安全读写重试间隔（秒）

# ---------------------------------------------------------------------------
# 展开参数
# ---------------------------------------------------------------------------
DEFAULT_UNFOLD_ANGLE = 45.0         # 展开角度（度）
DEFAULT_UNFOLD_TIMEOUT = 6.0        # 展开单次运动超时（秒）
UNFOLD_OVERLOAD_SETTLE = 0.3        # 展开时堵转后关扭矩等待（秒）
UNFOLD_TOLERANCE_DEG = 5.0          # 展开到位判断：误差在此度数内视为成功

# ---------------------------------------------------------------------------
# 校准/展开顺序（SO 六轴臂）
# ---------------------------------------------------------------------------
CALIBRATE_FIRST: list[str] = ["shoulder_pan"]
CALIBRATE_REST: list[str] = [
    "wrist_roll", "gripper", "wrist_flex", "elbow_flex", "shoulder_lift"
]
UNFOLD_ORDER: list[str] = ["wrist_flex", "elbow_flex", "shoulder_lift"]
