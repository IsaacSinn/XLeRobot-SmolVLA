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
飞特 STS 舵机自动校准（含展开）。

完整流程（一条命令）：
  阶段0  初始化：停止全部舵机, Lock=1, 配置 PID/加速度, 上力矩
  阶段2  展开 2-4 号关节（可通过 --unfold-angle 0 跳过）
  阶段3  校准 2–6 号舵机 (5→6→4→3→2)
  阶段4  最后校准 1 号 shoulder_pan 并回中
  阶段5  等待用户确认后释放扭矩

用法示例：

  lerobot-auto-calibrate-feetech --port COM3
  lerobot-auto-calibrate-feetech --port COM3 --save
  lerobot-auto-calibrate-feetech --port COM3 --unfold-angle 0
  lerobot-auto-calibrate-feetech --port COM3 --save --robot-id default
  lerobot-auto-calibrate-feetech --port COM3 --unfold-only   # 仅调试臂展开（阶段0+阶段2）
"""

import argparse
import sys
import time
from collections.abc import Callable

import draccus
from lerobot.motors import MotorCalibration
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION
from lerobot.motors.feetech import COMM_ERR, FeetechMotorsBus
from lerobot.motors.feetech.calibration_defaults import (
    CALIBRATE_FIRST,
    CALIBRATE_REST,
    DEFAULT_ACCELERATION,
    FULL_TURN,
    DEFAULT_D_COEFFICIENT,
    DEFAULT_I_COEFFICIENT,
    DEFAULT_MAX_TORQUE,
    DEFAULT_P_COEFFICIENT,
    DEFAULT_POS_SPEED,
    DEFAULT_TIMEOUT,
    DEFAULT_TORQUE_LIMIT,
    DEFAULT_UNFOLD_ANGLE,
    DEFAULT_UNFOLD_TIMEOUT,
    DEFAULT_VELOCITY_LIMIT,
    HOMING_OFFSET_MAX_MAG,
    MOTOR_NAMES,
    SO_FOLLOWER_MOTORS,
    STS_HALF_TURN_RAW,
    UNFOLD_ORDER,
    motor_label,
)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="飞特舵机自动校准（含展开）：一条命令完成全部流程。"
    )
    parser.add_argument(
        "--port", type=str, required=True,
        help="串口路径，例如 COM3 或 /dev/ttyUSB0",
    )
    parser.add_argument(
        "--motor", type=str, choices=MOTOR_NAMES, default=None,
        help="仅测试该舵机（跳过展开）；不指定则依次测试全部 6 个舵机",
    )

    cal = parser.add_argument_group("校准参数")
    cal.add_argument(
        "--velocity-limit", type=int, default=DEFAULT_VELOCITY_LIMIT,
        help=f"校准探测限位速度（恒速模式 Goal_Velocity），默认 {DEFAULT_VELOCITY_LIMIT}",
    )
    cal.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT,
        help=f"校准单方向限位等待超时（秒），默认 {DEFAULT_TIMEOUT}",
    )
    unfold = parser.add_argument_group("展开参数")
    unfold.add_argument(
        "--unfold-only", action="store_true",
        help="仅执行臂展开（阶段0 初始化 + 阶段2 展开），不校准，用于调试展开逻辑",
    )
    unfold.add_argument(
        "--unfold-angle", type=float, default=DEFAULT_UNFOLD_ANGLE,
        help=f"展开角度（度），设为 0 跳过展开。默认 {DEFAULT_UNFOLD_ANGLE}",
    )
    unfold.add_argument(
        "--unfold-timeout", type=float, default=DEFAULT_UNFOLD_TIMEOUT,
        help=f"展开单次运动等待超时（秒），默认 {DEFAULT_UNFOLD_TIMEOUT}",
    )
    out = parser.add_argument_group("输出（与手动校准相同路径与格式）")
    out.add_argument(
        "--save", action="store_true",
        help="将校准数据写入舵机 EEPROM，并保存到与手动校准相同的本地路径（draccus 格式）",
    )
    out.add_argument(
        "--robot-id", type=str, default="default",
        help="保存时的机器人 id，对应路径 .../calibration/robots/<robot_type>/<robot_id>.json，与机械臂启动时 config.id 一致",
    )
    out.add_argument(
        "--robot-type", type=str, default="so_follower",
        choices=["so_follower", "so_leader"],
        help="Robot type for calibration file path: 'so_follower' (default) or 'so_leader'",
    )

    return parser.parse_args()


# ====================== 展开相关 ======================

def _unfold_joints(
    bus: FeetechMotorsBus,
    unfold_angle: float,
    unfold_timeout: float,
    unfold_directions: dict[str, str | None] | None = None,
) -> None:
    """展开 2-4 号关节，避免校准时机械干涉。若传入 unfold_directions 则记录每关节展开方向。"""
    print(f"\n{'='*20} 阶段2：展开 2-4 号关节 ({unfold_angle}°) {'='*20}")
    for motor in UNFOLD_ORDER:
        direction, _ = bus.unfold_single_joint(motor, unfold_angle, unfold_timeout)
        if unfold_directions is not None and direction is not None:
            unfold_directions[motor] = direction
    print("\n  展开完成，2-4 号关节已抬起。每个关节的展开方向如下：")
    if unfold_directions is not None:
        for motor in UNFOLD_ORDER:
            direction = unfold_directions.get(motor, "未知")
            print(f"    {motor_label(motor)}: 展开方向 = {direction}")


def _fold_arm(
    bus: FeetechMotorsBus,
    all_mins: dict[str, int],
    all_maxes: dict[str, int],
    all_unfold_directions: dict[str, str | None],
    *,
    motors: list[str] | None = None,
    unfold: bool = False,
    unfold_per_motor: dict[str, bool] | None = None,
) -> None:
    """将指定关节折叠或完全展开。多舵机同时运动。

    折叠（unfold=False）：forward 展开 → 折叠目标 range_max；reverse → range_min；夹爪固定 range_min。
    展开（unfold=True）：目标与折叠相反，forward → range_min，reverse → range_max；夹爪固定 range_max。
    motors: 要动的舵机列表；为 None 或空时使用默认顺序（shoulder_lift→elbow_flex→wrist_flex→gripper）。
    unfold_per_motor: 可选，按关节指定折叠(False)/展开(True)；未列出的关节用 unfold。为 None 时全部用 unfold。
    """
    default_order = ["shoulder_lift", "elbow_flex", "wrist_flex", "gripper"]
    fold_order = default_order if not motors else motors
    title = "折叠/展开机械臂" if unfold_per_motor else ("展开" if unfold else "折叠") + "机械臂"
    print(f"\n{'='*20} {title}（同时运动） {'='*20}")
    values: dict[str, tuple[int, int, int]] = {}

    for motor in fold_order:
        if motor not in all_mins or motor not in all_maxes:
            continue
        per_unfold = unfold_per_motor.get(motor, unfold) if unfold_per_motor is not None else unfold
        direction = all_unfold_directions.get(motor)
        # 先算折叠端、展开端，再按 per_unfold 选一端
        if motor == "gripper":
            fold_end = all_mins[motor]
            unfold_end = all_maxes[motor]
        else:
            fold_end = all_maxes[motor] if direction == "reverse" else all_mins[motor]
            unfold_end = all_mins[motor] if direction == "reverse" else all_maxes[motor]
        target = unfold_end if per_unfold else fold_end
        label = "range_max" if target == all_maxes[motor] else "range_min"
        if motor == "gripper":
            label += "(夹爪)" if per_unfold else "(夹爪正方向)"
        action_m = "展开" if per_unfold else "折叠"
        values[motor] = (target, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION)
        bus.write("Operating_Mode", motor, 0)  # 伺服模式
        try:
            pos = bus.read("Present_Position", motor, normalize=False)
            print(f"  {motor_label(motor)} 当前位置={pos}，{action_m}至 {label}={target}。")
        except COMM_ERR:
            print(f"  {motor_label(motor)} 读当前位置失败，{action_m}至 {label}={target}。")
    if not values:
        action = "展开" if unfold else "折叠"
        print(f"  无有效舵机，跳过{action}。\n")
        return
    bus.sync_write_pos_ex(values)
    time.sleep(0.3)
    # 轮询直到全部停止
    timeout_s = 10.0
    poll_s = 0.05
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        try:
            if all(bus.read("Moving", m, normalize=False) == 0 for m in values):
                break
        except COMM_ERR:
            pass
        time.sleep(poll_s)
    done_label = "折叠/展开" if unfold_per_motor else ("展开" if unfold else "折叠")
    for m in values:
        try:
            pos = bus.read("Present_Position", m, normalize=False)
            print(f"  {motor_label(m)} {done_label}后 结束位置={pos}，已到位")
        except COMM_ERR:
            print(f"  {motor_label(m)} {done_label}后 读位置失败，已到位")
    print(f"  {done_label}完成。\n")


def _move_arm_by_angle(
    bus: FeetechMotorsBus,
    all_unfold_directions: dict[str, str | None],
    angle_deg: float,
    *,
    fold: bool = False,
    motors: list[str] | None = None,
    all_mins: dict[str, int] | None = None,
    all_maxes: dict[str, int] | None = None,
) -> None:
    """相对当前位置，沿展开方向或折叠方向移动指定度数。不探测方向，依赖 all_unfold_directions。

    方向与 _fold_arm 一致：forward 展开 → 位置增加为展开、减少为折叠；reverse 展开 → 位置减少为展开、增加为折叠。
    fold: False=展开方向，True=折叠方向。
    motors: 要动的舵机列表，None 或空则使用默认顺序（shoulder_lift→elbow_flex→wrist_flex）。
    all_mins/all_maxes: 可选，有则对目标位置夹限。
    """
    default_order = ["shoulder_lift", "elbow_flex", "wrist_flex"]
    move_order = default_order if not motors else motors
    angle_steps = int(angle_deg / 360.0 * FULL_TURN)
    direction_label = "折叠" if fold else "展开"
    print(f"\n{'='*20} 相对当前位 {direction_label} {angle_deg:.1f}° {'='*20}")
    for motor in move_order:
        if all_mins is not None and all_maxes is not None and (motor not in all_mins or motor not in all_maxes):
            continue
        try:
            present = bus.read("Present_Position", motor, normalize=False)
        except COMM_ERR:
            print(f"  警告: {motor_label(motor)} 读当前位置失败，跳过")
            continue
        direction = all_unfold_directions.get(motor)
        # 与 _fold_arm 一致：forward → 展开为位置增加、折叠为位置减少；reverse → 反之
        if fold:
            target = present - angle_steps if direction == "forward" else present + angle_steps
        else:
            target = present + angle_steps if direction == "forward" else present - angle_steps
        if all_mins is not None and all_maxes is not None and motor in all_mins and motor in all_maxes:
            target = max(all_mins[motor], min(all_maxes[motor], target))
        print(f"  {motor_label(motor)} {direction_label} {angle_deg:.1f}°: pos {present} → {target}")
        ok = bus.write_pos_ex_and_wait(
            motor, target, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION,
            timeout_s=DEFAULT_UNFOLD_TIMEOUT, poll_interval_s=0.05,
        )
        if not ok:
            print(f"  警告: {motor_label(motor)} 运动超时，保持当前位置")
        else:
            print(f"  {motor_label(motor)} 已到位")
    print(f"  {direction_label}完成。\n")


# ====================== 校准相关 ======================


def _record_reference_position(
    bus: FeetechMotorsBus,
    motor_name: str,
    out: dict[str, int],
) -> None:
    """读取该舵机当前参考位置 (Present_Position + Homing_Offset) % FULL_TURN 并写入 out[motor_name]，读失败则不改 out。"""
    try:
        pr = bus.read("Present_Position", motor_name, normalize=False)
        ho = bus.read("Homing_Offset", motor_name, normalize=False)
        out[motor_name] = (pr + ho) % FULL_TURN
    except COMM_ERR:
        pass


def _calibrate_motors(
    bus: FeetechMotorsBus,
    motor_names: list[str],
    *,
    velocity_limit: int = DEFAULT_VELOCITY_LIMIT,
    timeout_s: float = DEFAULT_TIMEOUT,
    ccw_first: bool = False,
    unfold_directions: dict[str, str | None] | None = None,
    reference_positions: dict[str, int] | None = None,
) -> dict[str, tuple[int, int, int]]:
    """校准一组舵机（统一走 measure_ranges_of_motion_multi 后写回并回中）。返回 {电机名: (range_min, range_max, mid_raw)}。
    若提供 unfold_directions 且同时校准 2、3 号：两舵机一起动，2 号完全折叠、3 号往展开方向完全展开；否则按原逻辑回中。
    若提供 reference_positions：对应舵机校准时用该参考位置选弧，跳过限位回退与再读。"""
    if not motor_names:
        return {}
    raw_results = bus.measure_ranges_of_motion_multi(
        motor_names,
        velocity_limit=velocity_limit,
        timeout_s=timeout_s,
        ccw_first=ccw_first,
        reference_positions=reference_positions,
    )
    print("准备写入寄存器")
    result: dict[str, tuple[int, int, int]] = {}
    for m in motor_names:
        rmin, rmax, mid_raw, _raw_min_meas, _raw_max_meas, homing_offset = raw_results[m]
        print(f"  {motor_label(m)}: 偏移后 range_min={rmin}, range_max={rmax}, 中值={mid_raw}, 偏移寄存器 Homing_Offset={homing_offset}")
        time.sleep(0.05)
        try:
            ho_before = bus.read("Homing_Offset", m, normalize=False)
            min_before = bus.read("Min_Position_Limit", m, normalize=False)
            max_before = bus.read("Max_Position_Limit", m, normalize=False)
            print(f"  {motor_label(m)} 写入前: Min_Position_Limit={min_before}, Max_Position_Limit={max_before}, Homing_Offset={ho_before}")
        except COMM_ERR:
            print(f"  {motor_label(m)} 写入前: 读寄存器失败")
        bus.safe_write("Homing_Offset", m, homing_offset, normalize=False)
        bus.safe_write_position_limits(m, rmin, rmax)
        time.sleep(0.1)
        try:
            ho_after = bus.read("Homing_Offset", m, normalize=False)
            min_after = bus.read("Min_Position_Limit", m, normalize=False)
            max_after = bus.read("Max_Position_Limit", m, normalize=False)
            print(f"  {motor_label(m)} 写入后: Min_Position_Limit={min_after}, Max_Position_Limit={max_after}, Homing_Offset={ho_after}")
        except COMM_ERR:
            print(f"  {motor_label(m)} 写入后: 读寄存器失败")
        time.sleep(0.1)
        do_2_3_together = (
            unfold_directions is not None
            and "shoulder_lift" in motor_names
            and "elbow_flex" in motor_names
        )
        if m == "wrist_roll":
            pass
        elif do_2_3_together and m in ("shoulder_lift", "elbow_flex"):
            # 2、3 号锁定扭矩
            pass
        else:
            bus.go_to_mid(m)
        result[m] = (rmin, rmax, mid_raw)

    return result


# ====================== 连接与初始化（共享） ======================

def _connect_and_clear(port: str) -> FeetechMotorsBus:
    """创建 bus、清除残留 Overload 后正式连接。失败时抛出异常。"""
    bus = FeetechMotorsBus(port=port, motors=SO_FOLLOWER_MOTORS.copy())
    bus.connect(handshake=False)
    print("正在清除舵机残留状态...")
    all_zero = {m: 0 for m in MOTOR_NAMES}
    for _ in range(3):
        try:
            bus.sync_write("Goal_Velocity", all_zero)
        except COMM_ERR:
            pass
        try:
            bus.sync_write("Torque_Enable", all_zero)
        except COMM_ERR:
            pass
        time.sleep(0.2)
    bus.disconnect(disable_torque=False)
    time.sleep(0.2)
    bus.connect()
    print("所有舵机已就绪。")
    return bus


def _run_with_bus(
    port: str,
    interactive: bool,
    body: Callable[[FeetechMotorsBus], None],
) -> int:
    """连接 bus 后执行 body(bus)，统一处理连接失败、KeyboardInterrupt、Exception 与 disconnect。返回 0 成功，1 错误，130 用户中断。"""
    try:
        bus = _connect_and_clear(port)
    except Exception as e:
        print(f"连接失败: {e}", file=sys.stderr)
        return 1
    try:
        body(bus)
    except KeyboardInterrupt:
        print("\n用户中断，释放所有舵机...")
        bus.safe_disable_all()
        return 130
    except Exception as e:
        print(f"异常: {e}", file=sys.stderr)
        bus.safe_disable_all()
        if interactive:
            try:
                input("按回车退出...")
            except EOFError:
                pass
        return 1
    finally:
        bus.disconnect()
    return 0


# 阶段0 初始化：逐寄存器写→读→比较，配置表驱动；特殊项（限位、Torque_Enable）单独处理
INIT_CHECKS = [
    ("Lock", 1),
    ("Return_Delay_Time", 0),
    ("Operating_Mode", 0),
    ("Max_Torque_Limit", DEFAULT_MAX_TORQUE),
    ("Torque_Limit", DEFAULT_TORQUE_LIMIT),
    ("Acceleration", DEFAULT_ACCELERATION),
    ("P_Coefficient", DEFAULT_P_COEFFICIENT),
    ("I_Coefficient", DEFAULT_I_COEFFICIENT),
    ("D_Coefficient", DEFAULT_D_COEFFICIENT),
    ("Homing_Offset", 0),
]


def _run_init(bus: FeetechMotorsBus, *, interactive: bool = True) -> None:
    """阶段0：Lock=1、PID、限位、Homing_Offset、上力矩。参数异常时若 interactive 则等待回车。"""
    print(f"\n{'='*20} 阶段0：初始化 {'='*20}")
    for m in MOTOR_NAMES:
        print(f"正在配置舵机: {motor_label(m)}")
        try:
            bus.write("Torque_Enable", m, 0)
            time.sleep(0.05)
        except COMM_ERR:
            pass
        param_set_ok = True
        try:
            for reg, expected in INIT_CHECKS:
                bus.write(reg, m, expected, normalize=(reg != "Homing_Offset"))
                time.sleep(0.01)
                got = bus.read(reg, m, normalize=False)
                if got != expected:
                    print(f"  [警告] {reg} 设置失败于 {m}：设置值={expected}，读到值={got}")
                    param_set_ok = False
            # 限位：单独写/读/比较
            bus.write_position_limits(m, 0, 4095)
            time.sleep(0.05)
            limits = bus.read_position_limits(m)
            if limits != (0, 4095):
                print(f"  [警告] Position_Limits 设置失败于 {m}：设置值=(0, 4095)，读到值={limits}")
                param_set_ok = False
            time.sleep(0.2)
            # 最后上力矩
            bus.write("Torque_Enable", m, 1)
            time.sleep(0.05)
            te_read = bus.read("Torque_Enable", m, normalize=False)
            if te_read != 1:
                print(f"  [警告] Torque_Enable 使能失败于 {m}：设置值=1，读到值={te_read}")
                param_set_ok = False
            time.sleep(0.1)
        except Exception as e:
            print(f"  [异常] 设置参数时发生异常于 {m}: {e}")
            param_set_ok = False
        if not param_set_ok and interactive:
            try:
                input("  [警告] 参数设置/确认存在异常，检查连线与电源，按回车强制继续...")
            except Exception:
                pass
    print(
        f"已初始化并上力矩（P={DEFAULT_P_COEFFICIENT}, "
        f"Acc={DEFAULT_ACCELERATION}, Torque={DEFAULT_TORQUE_LIMIT}）。"
    )


# ====================== 对外入口（完整校准 / 仅展开 / 单舵机） ======================


def _apply_calibration_results(
    results: dict[str, tuple[int, int, int]],
    all_mins: dict[str, int],
    all_maxes: dict[str, int],
    all_mids: dict[str, int],
    motor_list: list[str],
) -> None:
    """将 _calibrate_motors 的返回写入 all_mins / all_maxes / all_mids。"""
    for m in motor_list:
        all_mins[m], all_maxes[m], all_mids[m] = results[m]


def run_full_calibration(
    port: str,
    *,
    save: bool = False,
    robot_id: str = "default",
    robot_type: str = "so_follower",
    velocity_limit: int = DEFAULT_VELOCITY_LIMIT,
    timeout_s: float = DEFAULT_TIMEOUT,
    unfold_timeout_s: float = DEFAULT_UNFOLD_TIMEOUT,
    interactive: bool = True,
) -> int:
    """完整校准流程：初始化 → 2–6 号舵机（含抬臂避障）→ 1 号 shoulder_pan 最后校准 → 折叠。
    若 save 为 True：写入舵机 EEPROM，并保存到与手动校准相同的路径与格式（draccus，供机械臂启动时加载）。

    供 CLI 或遥操等程序调用。返回 0 成功，1 错误，130 用户中断。
    """

    def body(bus: FeetechMotorsBus) -> None:
        all_mins: dict[str, int] = {}
        all_maxes: dict[str, int] = {}
        all_mids: dict[str, int] = {}
        all_unfold_directions: dict[str, str | None] = {}
        all_reference_positions: dict[str, int] = {}
        _run_init(bus, interactive=interactive)
        # 抬起4号舵机80度
        direction, _ = bus.unfold_single_joint("wrist_flex", 80, unfold_timeout_s)
        if direction is not None:
            all_unfold_directions["wrist_flex"] = direction
        time.sleep(0.1)
        # 抬起2、3号舵机并记录参考位置（Present_Position + Homing_Offset，用于校准时选弧）
        direction, _ = bus.unfold_single_joint("shoulder_lift", 15, unfold_timeout_s)
        if direction is not None:
            all_unfold_directions["shoulder_lift"] = direction
        _record_reference_position(bus, "shoulder_lift", all_reference_positions)
        direction, _ = bus.unfold_single_joint("elbow_flex", 30, unfold_timeout_s)
        if direction is not None:
            all_unfold_directions["elbow_flex"] = direction
        _record_reference_position(bus, "elbow_flex", all_reference_positions)
        time.sleep(0.1)
        # 折叠：shoulder_lift、elbow_flex 收回
        for m in ["shoulder_lift", "elbow_flex"]:
            bus.go_to_mid(m)
            time.sleep(0.1)
        # 使用多舵机校准 2、3 号舵机，首次旋转方向为各自抬起方向的反方向
        # forward 抬起 -> 先 CCW；reverse 抬起 -> 先 CW；未记录时默认先 CCW
        ccw_first_2_3 = {
            "shoulder_lift": all_unfold_directions.get("shoulder_lift") != "reverse",
            "elbow_flex": all_unfold_directions.get("elbow_flex") != "reverse",
        }
        print(f"\n{'='*20} 校准 2、3 号舵机（多舵机，先抬起的反方向） {'='*20}")
        results_2_3 = _calibrate_motors(
            bus, ["shoulder_lift", "elbow_flex"],
            velocity_limit=velocity_limit,
            timeout_s=timeout_s,
            ccw_first=ccw_first_2_3,
            unfold_directions=all_unfold_directions,
            reference_positions=all_reference_positions,
        )
        _apply_calibration_results(results_2_3, all_mins, all_maxes, all_mids, ["shoulder_lift", "elbow_flex"])
        _fold_arm(bus, all_mins, all_maxes, all_unfold_directions, motors=["shoulder_lift", "elbow_flex"])
      
        time.sleep(0.1)
        # 阶段3：校准其余 4、5、6 号舵机（多舵机同时校准，含抬臂避障）
        print(f"\n{'='*20} 阶段3：校准 4–6 号舵机（多舵机同时） {'='*20}")
        _move_arm_by_angle(bus, all_unfold_directions, 80, fold=False, motors=["elbow_flex"], all_mins=all_mins, all_maxes=all_maxes)
        CALIBRATE_REST_REMAINING = ["wrist_roll", "gripper", "wrist_flex"]
        results_rest = _calibrate_motors(
            bus, CALIBRATE_REST_REMAINING,
            velocity_limit=velocity_limit,
            timeout_s=timeout_s,
            reference_positions=all_reference_positions,
        )
        _apply_calibration_results(results_rest, all_mins, all_maxes, all_mids, CALIBRATE_REST_REMAINING)
        time.sleep(0.1)
        # 折叠 3 号、完全展开 4 号（一次调用同时执行）
        _fold_arm(bus, all_mins, all_maxes, all_unfold_directions, 
            motors=["elbow_flex", "wrist_flex","gripper"], 
            unfold_per_motor={"elbow_flex": False, "wrist_flex": True, "gripper": False})
        # 阶段4：最后校准 1 号舵机 shoulder_pan
        print(f"\n{'='*20} 阶段4：校准 {motor_label('shoulder_pan')}（1号）并回中 {'='*20}")
        results_pan = _calibrate_motors(
            bus, ["shoulder_pan"], velocity_limit=velocity_limit, timeout_s=timeout_s
        )
        _apply_calibration_results(results_pan, all_mins, all_maxes, all_mids, ["shoulder_pan"])
        time.sleep(0.1)
        motors_calibrated = CALIBRATE_REST + CALIBRATE_FIRST
        print(f"\n{'='*20} 校准结果 {'='*20}")
        for name in motors_calibrated:
            offset = all_mids[name] - STS_HALF_TURN_RAW
            print(
                f"  {motor_label(name)}: min={all_mins[name]}, max={all_maxes[name]}, "
                f"中间值={all_mids[name]}, 偏移量={offset}"
            )
     

        _fold_arm(bus, all_mins, all_maxes, all_unfold_directions)
           # 持久化前：解锁 EEPROM（Lock=0）并将所有舵机恢复为伺服模式（Operating_Mode=0）
        for name in bus.motors:
            bus.write("Lock", name, 0)
            time.sleep(0.01)
            bus.write("Operating_Mode", name, 0)
            time.sleep(0.01)
        time.sleep(1)
        if interactive:
            bus.safe_disable_all()
            print("\n校准结束。")
            
        if save:
            print(f"\n{'='*20} 持久化（与手动校准相同方案） {'='*20}")
            bus.safe_disable_all()
            cal = {}
            for name in motors_calibrated:
                m = SO_FOLLOWER_MOTORS[name]
                offset = all_mids[name] - STS_HALF_TURN_RAW
                offset = max(-HOMING_OFFSET_MAX_MAG, min(HOMING_OFFSET_MAX_MAG, offset))
                cal[name] = MotorCalibration(
                    id=m.id,
                    drive_mode=0,
                    homing_offset=offset,
                    range_min=all_mins[name],
                    range_max=all_maxes[name],
                )
            bus.write_calibration(cal, cache=True)
            print("已写入校准到舵机 EEPROM。")
            # 与手动校准相同路径与格式，供机械臂启动时加载
            calibration_fpath = HF_LEROBOT_CALIBRATION / "robots" / robot_type / f"{robot_id}.json"
            calibration_fpath.parent.mkdir(parents=True, exist_ok=True)
            with open(calibration_fpath, "w") as f, draccus.config_type("json"):
                draccus.dump(cal, f, indent=4)
            print(f"已写入校准到: {calibration_fpath}")
        print("释放所有舵机...")
        bus.safe_disable_all()

    return _run_with_bus(port, interactive, body)


def unfold_joints(
    port: str,
    angle_deg: float,
    *,
    timeout_s: float = DEFAULT_UNFOLD_TIMEOUT,
    interactive: bool = True,
) -> int:
    """仅做阶段0 初始化 + 展开 2–4 号关节到指定角度。用于调试展开。返回 0/1/130。"""

    def body(bus: FeetechMotorsBus) -> None:
        _run_init(bus, interactive=interactive)
        all_unfold_directions: dict[str, str | None] = {}
        if angle_deg > 0:
            _unfold_joints(bus, angle_deg, timeout_s, all_unfold_directions)
            print("  展开完成，展开方向如下：")
            for motor, direction in all_unfold_directions.items():
                print(f"    {motor_label(motor)}: {direction}")
            if interactive:
                input("  按回车释放扭矩并退出...")
        else:
            print("  展开角度为 0，未执行展开。")
            if interactive:
                input("  按回车释放扭矩并退出...")
        bus.safe_disable_all()

    return _run_with_bus(port, interactive, body)


def calibrate_single_motor(
    port: str,
    motor_name: str,
    *,
    velocity_limit: int = DEFAULT_VELOCITY_LIMIT,
    timeout_s: float = DEFAULT_TIMEOUT,
    interactive: bool = True,
) -> int:
    """仅做阶段0 + 校准指定舵机，不折叠、不保存。用于测试。返回 0/1/130。"""

    def body(bus: FeetechMotorsBus) -> None:
        _run_init(bus, interactive=interactive)
        print(f"\n{'='*20} 校准 {motor_label(motor_name)} {'='*20}")
        _calibrate_motors(bus, [motor_name], velocity_limit=velocity_limit, timeout_s=timeout_s)
        time.sleep(0.1)
        if interactive:
            input("  校准完成，按回车释放扭矩并退出...")
        bus.safe_disable_all()

    return _run_with_bus(port, interactive, body)


# ====================== CLI 入口 ======================

def main() -> int:
    """CLI：根据参数调用完整校准、仅展开或单舵机校准。"""
    args = parse_args()
    print(f"串口: {args.port}")
    if getattr(args, "unfold_only", False):
        print("仅臂展开（--unfold-only）：阶段0 初始化 + 阶段2 展开，不校准")
        print(f"展开角度: {args.unfold_angle}°")
        return unfold_joints(
            args.port,
            args.unfold_angle,
            timeout_s=args.unfold_timeout,
            interactive=True,
        )
    if args.motor is not None:
        print(f"单舵机模式: {args.motor}")
        return calibrate_single_motor(
            args.port,
            args.motor,
            velocity_limit=args.velocity_limit,
            timeout_s=args.timeout,
            interactive=True,
        )
    print(f"完整校准: {CALIBRATE_FIRST + CALIBRATE_REST}")
    return run_full_calibration(
        args.port,
        save=args.save,
        robot_id=args.robot_id,
        robot_type=args.robot_type,
        velocity_limit=args.velocity_limit,
        timeout_s=args.timeout,
        unfold_timeout_s=args.unfold_timeout,
        interactive=True,
    )


if __name__ == "__main__":
    sys.exit(main())
