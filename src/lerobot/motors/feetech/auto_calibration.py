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

"""Feetech 自动校准与 WritePosEx 相关逻辑（测机械范围、默认 range、单条写位置+速度+加速度）。"""

import logging
import time
import scservo_sdk as scs
from ..motors_bus import NameOrID

from .calibration_defaults import (
    DEFAULT_ACCELERATION,
    DEFAULT_TIMEOUT,
    DEFAULT_POS_SPEED,
    FULL_TURN,
    MID_POS,
    OVERLOAD_SETTLE_TIME,
    POSITION_TOLERANCE,
    SAFE_IO_INTERVAL,
    SAFE_IO_RETRIES,
    SO_STS_DEFAULT_RANGES,
    STALL_POSITION_DELTA_THRESHOLD,
    STALL_VELOCITY_THRESHOLD,
    UNFOLD_OVERLOAD_SETTLE,
    UNFOLD_TOLERANCE_DEG,
    HOMING_OFFSET_MAX_MAG,
    motor_label,
)

COMM_ERR = (RuntimeError, ConnectionError)
"""舵机通信可能抛出的异常类型：RuntimeError(Overload) 和 ConnectionError(no status packet)。"""

logger = logging.getLogger(__name__)


class FeetechCalibrationMixin:
    """提供自动测机械范围、默认 range、write_pos_ex_and_wait/wait_until_stopped。"""

    # 一条指令写 Acceleration(41) + Goal_Position(42-43) + Goal_Time(44-45) + Goal_Velocity(46-47)，共 7 字节（与 STServo WritePosEx 一致）
    _POS_EX_START_ADDR = 41
    _POS_EX_LEN = 7
    # Min_Position_Limit(9,2) + Max_Position_Limit(11,2) 连续 4 字节，可一条指令写完
    _POS_LIMITS_START_ADDR = 9
    _POS_LIMITS_LEN = 4

    def get_default_range(self, motor: NameOrID) -> tuple[int, int]:
        """返回指定电机自动校准时使用的默认 (range_min, range_max)。

        若电机名在 calibration_defaults.SO_STS_DEFAULT_RANGES 中则使用预设值，
        否则使用该型号分辨率下的全量程 (0, max_res)。
        """
        motor_names = self._get_motors_list(motor)
        name = motor_names[0]
        if name in SO_STS_DEFAULT_RANGES:
            return SO_STS_DEFAULT_RANGES[name]
        model = self._get_motor_model(motor)
        max_res = self.model_resolution_table[model] - 1
        return (0, max_res)

    def _wait_for_stall(
        self,
        motor: str,
        stall_confirm_samples: int,
        timeout_s: float,
        sample_interval_s: float,
        *,
        velocity_threshold: int = STALL_VELOCITY_THRESHOLD,
        position_delta_threshold: int = STALL_POSITION_DELTA_THRESHOLD,
    ) -> str:
        """轮询检测堵转/限位，返回停止原因字符串。

        优先判断 AND 条件（速度近零 + 位置稳定 + Moving=0），满足则立即返回；
        否则再判断 Status 寄存器 BIT5（过载）或通信异常，连续 stall_confirm_samples 次即判定。
        """
        stall_count = 0
        stable_count = 0
        prev_position: int | None = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            try:
                vel = self.read("Present_Velocity", motor, normalize=False)
                pos = self.read("Present_Position", motor, normalize=False)
                moving = self.read("Moving", motor, normalize=False)
                status = self.read("Status", motor, normalize=False)
            except COMM_ERR:
                stable_count = 0
                stall_count += 1
                if stall_count >= stall_confirm_samples:
                    return f"堵转确认({stall_confirm_samples}次): 通信异常"
                time.sleep(sample_interval_s)
                continue

            # 优先：速度近零 且 位置稳定 且 Moving=0（连续 N 次）
            vel_ok = abs(vel) < velocity_threshold
            pos_ok = (
                prev_position is None
                or abs(pos - prev_position) < position_delta_threshold
            )
            moving_ok = moving == 0
            if vel_ok and pos_ok and moving_ok:
                stable_count += 1
                if stable_count >= stall_confirm_samples:
                    return (
                        f"限位确认({stall_confirm_samples}次): "
                        "速度近零+位置稳定+Moving=0"
                    )
            else:
                stable_count = 0
            prev_position = pos

            # 其次：Status BIT5 过载
            if status & 0x20:
                stall_count += 1
                if stall_count >= stall_confirm_samples:
                    return f"堵转确认({stall_confirm_samples}次): Status=0x{status:02X}(BIT5过载)"
            else:
                stall_count = 0

            time.sleep(sample_interval_s)
        return f"超时({timeout_s}s)"

    def _wait_for_stall_multi(
        self,
        motors: list[str],
        stall_confirm_samples: int,
        timeout_s: float,
        sample_interval_s: float,
        *,
        velocity_threshold: int = STALL_VELOCITY_THRESHOLD,
        position_delta_threshold: int = STALL_POSITION_DELTA_THRESHOLD,
    ) -> tuple[dict[str, str], dict[str, int]]:
        """多舵机轮询堵转：同时监控多台，谁先堵转就给谁写 Goal_Velocity=0，全部堵转或超时后返回。

        返回 (reasons, positions)：每台停止原因与堵转时的 Present_Position。
        """
        still_running = set(motors)
        reasons: dict[str, str] = {}
        positions: dict[str, int] = {}
        # 每台状态：上一帧位置、稳定计数、过载计数
        prev_pos: dict[str, int | None] = {m: None for m in motors}
        stable_count: dict[str, int] = {m: 0 for m in motors}
        stall_count: dict[str, int] = {m: 0 for m in motors}
        t0 = time.monotonic()

        while still_running and (time.monotonic() - t0 < timeout_s):
            for m in list(still_running):
                try:
                    vel = self.read("Present_Velocity", m, normalize=False)
                    pos = self.read("Present_Position", m, normalize=False)
                    moving = self.read("Moving", m, normalize=False)
                    status = self.read("Status", m, normalize=False)
                except COMM_ERR:
                    stall_count[m] = stall_count.get(m, 0) + 1
                    if stall_count[m] >= stall_confirm_samples:
                        reasons[m] = f"堵转确认({stall_confirm_samples}次): 通信异常"
                        positions[m] = self._read_with_retry("Present_Position", m)
                        self.write("Goal_Velocity", m, 0)
                        still_running.discard(m)
                    continue

                vel_ok = abs(vel) < velocity_threshold
                pos_ok = (
                    prev_pos[m] is None
                    or abs(pos - prev_pos[m]) < position_delta_threshold
                )
                moving_ok = moving == 0
                if vel_ok and pos_ok and moving_ok:
                    stable_count[m] = stable_count.get(m, 0) + 1
                    if stable_count[m] >= stall_confirm_samples:
                        reasons[m] = (
                            f"限位确认({stall_confirm_samples}次): "
                            "速度近零+位置稳定+Moving=0"
                        )
                        positions[m] = pos
                        self.write("Goal_Velocity", m, 0)
                        still_running.discard(m)
                        continue
                else:
                    stable_count[m] = 0
                prev_pos[m] = pos

                if status & 0x20:
                    stall_count[m] = stall_count.get(m, 0) + 1
                    if stall_count[m] >= stall_confirm_samples:
                        reasons[m] = f"堵转确认({stall_confirm_samples}次): Status=0x{status:02X}(BIT5过载)"
                        positions[m] = pos
                        self.write("Goal_Velocity", m, 0)
                        still_running.discard(m)
                else:
                    stall_count[m] = 0

            time.sleep(sample_interval_s)

        for m in still_running:
            reasons[m] = f"超时({timeout_s}s)"
            try:
                positions[m] = self.read("Present_Position", m, normalize=False)
            except COMM_ERR:
                positions[m] = 0
            try:
                self.write("Goal_Velocity", m, 0)
            except COMM_ERR:
                pass
        return reasons, positions

    def _prepare_motors_for_range_measure(self, motors: list[str]) -> None:
        """为测范围做准备：清过载、关扭矩，设 Phase(BIT4=0)、Homing_Offset=0、恒速模式、上力矩。"""
        from .feetech import OperatingMode

        for m in motors:
            self._safe_stop_and_clear_overload(m)
          #  self.disable_torque(m)
        for m in motors:
            phase_raw = self.read("Phase", m, normalize=False)
            if phase_raw & 0x10:
                self.write("Phase", m, phase_raw & ~0x10, normalize=False)
                print(f"  [{motor_label(m)}] Phase(reg18): 0x{phase_raw:02X} -> 0x{phase_raw & ~0x10:02X} (BIT4=0 单圈)")
            else:
                print(f"  [{motor_label(m)}] Phase(reg18): 0x{phase_raw:02X} (已为单圈)")
            self.write("Homing_Offset", m, 0, normalize=False)
            self.write("Operating_Mode", m, OperatingMode.VELOCITY.value)
            mode = self.read("Operating_Mode", m, normalize=False)
            if len(motors) == 1:
                print(f"  [{motor_label(m)}] Operating_Mode={mode} (期望1, 恒速模式)")
            self.enable_torque(m)
        if motors:
            time.sleep(0.1)

    def _run_direction_until_stall(
        self,
        motors: list[str],
        velocity: int | dict[str, int],
        *,
        stall_confirm_samples: int = 2,
        timeout_s: float = 10.0,
        sample_interval_s: float = 0.05,
        initial_move_delay_s: float = 0.5,
    ) -> tuple[dict[str, str], dict[str, int]]:
        """一条指令让指定舵机运行，轮询堵转后停、清过载。velocity 可为同一速度(int)或每台速度(dict)。返回 (每台停止原因, 每台堵转位置)。"""
        vel_dict = velocity if isinstance(velocity, dict) else {m: velocity for m in motors}
        self.sync_write(
            "Goal_Velocity",
            vel_dict,
            normalize=False,
        )
        time.sleep(initial_move_delay_s)
        if len(motors) == 1:
            m = motors[0]
            reason = self._wait_for_stall(
                m, stall_confirm_samples, timeout_s, sample_interval_s
            )
            reasons = {m: reason}
            positions = {m: self._read_with_retry("Present_Position", m)}
        else:
            reasons, positions = self._wait_for_stall_multi(
                motors, stall_confirm_samples, timeout_s, sample_interval_s
            )
        # for m in motors:
        #     self._safe_stop_and_clear_overload(m)
        return reasons, positions

    def _compute_mid_and_range_from_limits(
        self,
        motor: str,
        pos_cw: int,
        pos_ccw: int,
        *,
        move_timeout: float = 5.0,
        reference_pos: int | None = None,
    ) -> tuple[int, int, int, int, int, int]:
        """从 CW/CCW 堵转位置做回退、算物理中值与范围。返回 (range_min, range_max, mid, raw_min, raw_max, homing_offset)。
        若传入 reference_pos（如抬起阶段采样的 (Present_Position+Homing_Offset)%FULL_TURN），则跳过回退与再读，直接用其选弧。"""
        arc_ccw_to_cw = (pos_cw - pos_ccw) % FULL_TURN
        arc_cw_to_ccw = (pos_ccw - pos_cw) % FULL_TURN
        if reference_pos is not None:
            start_pos = reference_pos
            print(
                f"  [{motor_label(motor)}] 使用预采参考位置 start_pos={start_pos}，跳过限位回退"
            )
        else:
            print("开始探测参考位置...")
            shortest_arc = min(arc_ccw_to_cw, arc_cw_to_ccw)
            steps_back = max(1, shortest_arc // 3)
            back_deg = steps_back * 360.0 / FULL_TURN
            print(
                f"  [{motor_label(motor)}] 长弧: {max(arc_ccw_to_cw, arc_cw_to_ccw)} 步, "
                f"短弧: {min(arc_ccw_to_cw, arc_cw_to_ccw)} 步, "
                f"回退步数: {steps_back} 步 ({back_deg:.1f}°)"
            )
            self.unfold_single_joint(motor, back_deg, move_timeout=move_timeout)
            time.sleep(0.1)
            present_raw = self._read_with_retry("Present_Position", motor)
            homing_raw = self._read_with_retry("Homing_Offset", motor)
            start_pos = (present_raw + homing_raw) % FULL_TURN
            print(
                f"  [{motor_label(motor)}] 从限位回退 {steps_back} 步 ({back_deg:.1f}°) "
                f"得到参考位置: present={present_raw}, offset={homing_raw}, 实际={start_pos}"
            )
        arc_ccw_to_cw = (pos_cw - pos_ccw) % FULL_TURN
        arc_cw_to_ccw = (pos_ccw - pos_cw) % FULL_TURN
        start_in_arc_a = (start_pos - pos_ccw) % FULL_TURN <= arc_ccw_to_cw
        if start_in_arc_a:
            physical_range = arc_ccw_to_cw
            mid = (pos_ccw + physical_range // 2) % FULL_TURN
        else:
            physical_range = arc_cw_to_ccw
            mid = (pos_cw + physical_range // 2) % FULL_TURN
        raw_min = min(pos_cw, pos_ccw)
        raw_max = max(pos_cw, pos_ccw)
        homing_offset = mid - MID_POS
        homing_offset = max(
            -HOMING_OFFSET_MAX_MAG,
            min(HOMING_OFFSET_MAX_MAG, homing_offset),
        )
        half = physical_range // 2
        range_min = max(0, min(FULL_TURN - 1, MID_POS - half))
        range_max = max(0, min(FULL_TURN - 1, MID_POS + half))
        crosses_zero = pos_ccw > pos_cw
        print(
            f"  [{motor_label(motor)}] CW={pos_cw}, CCW={pos_ccw}, 计算参考位置={start_pos}, "
            f"极值步长={physical_range}  ({physical_range * 360 / FULL_TURN:.1f}°), "
            f"物理中值={mid}, 过零点={crosses_zero}"
        )
        return range_min, range_max, mid, raw_min, raw_max, homing_offset

    def _safe_stop_and_clear_overload(self, motor: str, settle_s: float = 0.5) -> None:
        """堵转后安全停止：写 Goal_Velocity=0 → 关扭矩 → 等待 Overload/通信异常清除。"""
        for _ in range(5):
            try:
                self.write("Goal_Velocity", motor, 0)
                break
            except COMM_ERR:
                time.sleep(0.1)
        for _ in range(5):
            try:
                self.disable_torque(motor)
                break
            except COMM_ERR:
                time.sleep(0.1)
        time.sleep(settle_s)

    def _read_with_retry(
        self, data_name: str, motor: str, retries: int = 5, interval_s: float = 0.2
    ) -> int:
        """带重试的读取，用于 Overload/通信异常 恢复期间读寄存器。"""
        for i in range(retries):
            try:
                return self.read(data_name, motor, normalize=False)
            except COMM_ERR as e:
                if i < retries - 1:
                    time.sleep(interval_s)
                    continue
                raise RuntimeError(
                    f"_read_with_retry: {retries}次均失败 {data_name} on {motor}: {e}"
                ) from e
        raise RuntimeError(f"_read_with_retry: 无法读取 {data_name} on {motor}")

    def safe_read(
        self,
        reg: str,
        motor: NameOrID,
        *,
        retries: int = SAFE_IO_RETRIES,
        interval_s: float = SAFE_IO_INTERVAL,
    ) -> int:
        """带重试的安全读取。"""
        return self._read_with_retry(reg, self._get_motors_list(motor)[0], retries=retries, interval_s=interval_s)

    def safe_write(
        self,
        reg: str,
        motor: NameOrID,
        value: int,
        *,
        normalize: bool = True,
        retries: int = SAFE_IO_RETRIES,
        interval_s: float = SAFE_IO_INTERVAL,
    ) -> None:
        """带重试的安全写入。"""
        motor_name = self._get_motors_list(motor)[0]
        for i in range(retries):
            try:
                self.write(reg, motor_name, value, normalize=normalize)
                return
            except COMM_ERR as e:
                if i < retries - 1:
                    time.sleep(interval_s)
                    continue
                raise RuntimeError(
                    f"safe_write: {retries}次均失败 {reg}={value} on {motor_name}: {e}"
                ) from e
        raise RuntimeError(f"safe_write: 无法写入 {reg} on {motor_name}")

    def _write_torque_with_recovery(
        self, motor: str, value: int, retries: int = 3, interval_s: float = 0.5
    ) -> None:
        """写 Torque_Enable，根据异常类型分别恢复后重试。

        - RuntimeError（Overload）：先关扭矩清除过载状态，等待后重试。
        - ConnectionError（无应答）：直接等待后重试。
        超过重试次数仍失败则继续抛出。
        """
        for attempt in range(retries):
            try:
                self.write("Torque_Enable", motor, value)
                return
            except RuntimeError as e:
                # Overload：关扭矩清除过载，等待后重试
                if attempt < retries - 1:
                    print(
                        f"  [{motor_label(motor)}] Torque_Enable={value} Overload，清除后重试"
                        f"({attempt + 1}/{retries}): {e}"
                    )
                    try:
                        self.write("Torque_Enable", motor, 0)
                    except COMM_ERR:
                        pass
                    time.sleep(interval_s)
                else:
                    raise RuntimeError(
                        f"_write_torque_with_recovery: {retries}次均失败 Torque_Enable={value} on {motor}: {e}"
                    ) from e
            except ConnectionError as e:
                # 无应答：等待后重试
                if attempt < retries - 1:
                    print(
                        f"  [{motor_label(motor)}] Torque_Enable={value} 无应答，等待后重试"
                        f"({attempt + 1}/{retries}): {e}"
                    )
                    time.sleep(interval_s)
                else:
                    raise RuntimeError(
                        f"_write_torque_with_recovery: {retries}次均失败 Torque_Enable={value} on {motor}: {e}"
                    ) from e

    def _clear_and_enable_torque(self, motor: str, settle_s: float = OVERLOAD_SETTLE_TIME) -> None:
        """清除过载后重新上力矩：先关扭矩等待，再带恢复重试上力矩。

        用于堵转停止后需要反向运动的场合，替代裸 enable_torque 调用。
        """
        # 关扭矩清除过载状态（已有重试逻辑）
        for _ in range(5):
            try:
                self.write("Torque_Enable", motor, 0)
                break
            except COMM_ERR:
                time.sleep(0.1)
        time.sleep(settle_s)
        # 带恢复重试上力矩
        self._write_torque_with_recovery(motor, 1)
        try:
            self.write("Lock", motor, 1)
        except COMM_ERR:
            pass

    def safe_write_position_limits(
        self,
        motor: NameOrID,
        rmin: int,
        rmax: int,
        *,
        retries: int = SAFE_IO_RETRIES,
        interval_s: float = SAFE_IO_INTERVAL,
    ) -> None:
        """带重试的安全写入 Min/Max 位置限制（一条指令）。"""
        motor_name = self._get_motors_list(motor)[0]
        for i in range(retries):
            try:
                self.write_position_limits(motor_name, rmin, rmax)
                return
            except COMM_ERR as e:
                if i < retries - 1:
                    time.sleep(interval_s)
                    continue
                raise RuntimeError(
                    f"safe_write_position_limits: {retries}次均失败 rmin={rmin} rmax={rmax} on {motor_name}: {e}"
                ) from e
        raise RuntimeError(f"safe_write_position_limits: 无法写入 on {motor_name}")

    def safe_disable_all(
        self,
        motor_names: list[str] | None = None,
        *,
        num_try_per_motor: int = 3,
        interval_s: float = 0.1,
    ) -> None:
        """安全释放所有舵机扭矩，忽略通信异常。"""
        names = motor_names if motor_names is not None else list(self.motors.keys())
        for m in names:
            for _ in range(num_try_per_motor):
                try:
                    self.write("Torque_Enable", m, 0)
                    break
                except COMM_ERR:
                    time.sleep(interval_s)

    def go_to_mid(
        self,
        motor: NameOrID,
        *,
        timeout_s: float = DEFAULT_TIMEOUT,
        poll_interval_s: float = 0.05,
    ) -> bool:
        """让舵机回到中位（伺服模式），等待到位后返回。返回 True 表示到位，False 表示超时。"""
        motor_name = self._get_motors_list(motor)[0]
        ok = self.write_pos_ex_and_wait(
            motor_name,
            MID_POS,
            DEFAULT_POS_SPEED,
            DEFAULT_ACCELERATION,
            timeout_s=timeout_s,
            poll_interval_s=poll_interval_s,
        )
        try:
            cur = self.read("Present_Position", motor_name, normalize=False)
        except COMM_ERR:
            cur = -1
        if not ok:
            logger.warning(
                "%s 回中超时(%.1fs)，当前位置=%s",
                motor_label(motor_name),
                timeout_s,
                cur,
            )
        return ok

    def measure_ranges_of_motion(
        self,
        motor: NameOrID,
        *,
        velocity_limit: int = 1000,
        stall_confirm_samples: int = 2,
        timeout_s: float = 10.0,
        sample_interval_s: float = 0.05,
        initial_move_delay_s: float = 0.5,
    ) -> tuple[int, int, int, int, int, int]:
        """自动测量单个电机的机械限位范围（恒速模式）。

        仅允许传入一个 motor。扭矩（Max_Torque_Limit、Torque_Limit）和加速度（Acceleration）
        应在调用前统一初始化，本方法仅使用 Goal_Velocity 驱动。

        流程：
        1. 设置寄存器18 BIT4=0（单圈角度反馈，0-4095）
        2. 清零 Homing_Offset
        3. 切换到 Operating_Mode=1（恒速模式），Goal_Velocity 驱动正反转
        4. 通过 Status 寄存器 BIT5 检测堵转，得到 CW/CCW 限位
        5. 不依赖起始位置：从 CCW 限位沿“最后一次运行的反方向”回退若干步得到参考点 P。
           回退步数 = min(到0距离, 到4095距离) 的一半；若 CCW 极值恰为 0 或 4095 则用 CW 极值计算。

        Returns:
            (range_min, range_max, mid, raw_min, raw_max, homing_offset) 六个整数：
            - range_min, range_max: 偏移后空间下的极值（0~4095，不跨零），用于写 Min/Max 限位；
            - mid: 测量得到的物理中值（raw 编码）；
            - raw_min, raw_max: 测量得到的极值（raw 编码，可能跨零）；
            - homing_offset: 偏移量 mid - MID_POS，用于写 Homing_Offset。
        """
        motor = self._get_motors_list(motor)[0]
        self._prepare_motors_for_range_measure([motor])

        cw_reasons, pos_cw_dict = self._run_direction_until_stall(
            [motor],
            velocity_limit,
            stall_confirm_samples=stall_confirm_samples,
            timeout_s=timeout_s,
            sample_interval_s=sample_interval_s,
            initial_move_delay_s=initial_move_delay_s,
        )
        print(f"  [{motor_label(motor)}] CW 停止原因: {cw_reasons[motor]}")
        print(f"  [{motor_label(motor)}] CW 堵转位置: {pos_cw_dict[motor]}")

        self._clear_and_enable_torque(motor)
        time.sleep(0.05)
        ccw_reasons, pos_ccw_dict = self._run_direction_until_stall(
            [motor],
            -velocity_limit,
            stall_confirm_samples=stall_confirm_samples,
            timeout_s=timeout_s,
            sample_interval_s=sample_interval_s,
            initial_move_delay_s=initial_move_delay_s,
        )
        print(f"  [{motor_label(motor)}] CCW 停止原因: {ccw_reasons[motor]}")
        print(f"  [{motor_label(motor)}] CCW 堵转位置: {pos_ccw_dict[motor]}")

        return self._compute_mid_and_range_from_limits(
            motor, pos_cw_dict[motor], pos_ccw_dict[motor]
        )

    def measure_ranges_of_motion_multi(
        self,
        motors: list[str],
        *,
        velocity_limit: int = 1000,
        stall_confirm_samples: int = 2,
        timeout_s: float = 10.0,
        sample_interval_s: float = 0.05,
        initial_move_delay_s: float = 0.5,
        ccw_first: bool | dict[str, bool] = False,
        reference_positions: dict[str, int] | None = None,
    ) -> dict[str, tuple[int, int, int, int, int, int]]:
        """多舵机同时测量机械限位：同时启动、轮询堵转（谁先停就给谁写 0），全部停后读位置；每台单独做回退与中值计算。

        ccw_first: 为 True 时该舵机先 CCW 再 CW；为 dict 时每台舵机独立指定（motor_name -> True/False）。
        reference_positions: 若提供某电机的参考位置（(Present_Position+Homing_Offset)%FULL_TURN），则该校准跳过限位回退，直接用该位置选弧。
        返回 dict[电机名, (range_min, range_max, mid, raw_min, raw_max, homing_offset)]。
        """
        if not motors:
            return {}
        if len(motors) == 1:
            m = motors[0]
            t = self.measure_ranges_of_motion(
                m,
                velocity_limit=velocity_limit,
                stall_confirm_samples=stall_confirm_samples,
                timeout_s=timeout_s,
                sample_interval_s=sample_interval_s,
                initial_move_delay_s=initial_move_delay_s,
            )
            return {m: t}

        self._prepare_motors_for_range_measure(motors)

        # 每台舵机各自方向：先 CCW 或先 CW
        def _ccw_first(m: str) -> bool:
            return ccw_first.get(m, False) if isinstance(ccw_first, dict) else bool(ccw_first)

        first_vel_dict = {
            m: (-velocity_limit if _ccw_first(m) else velocity_limit) for m in motors
        }
        second_vel_dict = {
            m: (velocity_limit if _ccw_first(m) else -velocity_limit) for m in motors
        }

        first_reasons, first_pos = self._run_direction_until_stall(
            motors,
            first_vel_dict,
            stall_confirm_samples=stall_confirm_samples,
            timeout_s=timeout_s,
            sample_interval_s=sample_interval_s,
            initial_move_delay_s=initial_move_delay_s,
        )
        for m in motors:
            label = "CCW" if first_vel_dict[m] < 0 else "CW"
            print(f"  [{motor_label(m)}] {label} 停止原因: {first_reasons[m]}, 堵转位置: {first_pos[m]}")
        for m in motors:
            self._clear_and_enable_torque(m)
        time.sleep(0.05)
        second_reasons, second_pos = self._run_direction_until_stall(
            motors,
            second_vel_dict,
            stall_confirm_samples=stall_confirm_samples,
            timeout_s=timeout_s,
            sample_interval_s=sample_interval_s,
            initial_move_delay_s=initial_move_delay_s,
        )
        for m in motors:
            label = "CCW" if second_vel_dict[m] < 0 else "CW"
            print(f"  [{motor_label(m)}] {label} 停止原因: {second_reasons[m]}, 堵转位置: {second_pos[m]}")
        time.sleep(OVERLOAD_SETTLE_TIME)

        # 按每台的第一/二方向还原 pos_cw、pos_ccw（CW 为正速度限位，CCW 为负速度限位）
        pos_cw_dict = {
            m: (first_pos[m] if first_vel_dict[m] == velocity_limit else second_pos[m])
            for m in motors
        }
        pos_ccw_dict = {
            m: (second_pos[m] if first_vel_dict[m] == velocity_limit else first_pos[m])
            for m in motors
        }

        result: dict[str, tuple[int, int, int, int, int, int]] = {}
        for m in motors:
            ref_pos = reference_positions.get(m) if reference_positions else None
            result[m] = self._compute_mid_and_range_from_limits(
                m, pos_cw_dict[m], pos_ccw_dict[m], reference_pos=ref_pos
            )
        return result

    def _write_raw_bytes(
        self,
        addr: int,
        motor_id: int,
        data: list[int],
        *,
        num_retry: int = 0,
        raise_on_error: bool = True,
        err_msg: str = "",
    ) -> tuple[int, int]:
        """向指定寄存器起始地址写入原始字节序列（用于 write_pos_ex_and_wait、sync_write_pos_ex 等）。"""
        for n_try in range(1 + num_retry):
            comm, error = self.packet_handler.writeTxRx(
                self.port_handler, motor_id, addr, len(data), data
            )
            if self._is_comm_success(comm):
                break
            logger.debug(
                f"_write_raw_bytes @{addr} len={len(data)} id={motor_id} try={n_try}: "
                + self.packet_handler.getTxRxResult(comm)
            )
        if not self._is_comm_success(comm) and raise_on_error:
            raise ConnectionError(f"{err_msg} {self.packet_handler.getTxRxResult(comm)}")
        if self._is_error(error) and raise_on_error:
            raise RuntimeError(f"{err_msg} {self.packet_handler.getRxPacketError(error)}")
        return comm, error

    def write_position_limits(
        self,
        motor: NameOrID,
        rmin: int,
        rmax: int,
        *,
        num_retry: int = 0,
    ) -> None:
        """一条指令写入 Min_Position_Limit(9) + Max_Position_Limit(11)，共 4 字节。"""
        id_ = self._get_motor_id(motor)
        data = (
            self._split_into_byte_chunks(rmin, 2)
            + self._split_into_byte_chunks(rmax, 2)
        )
        err_msg = f"write_position_limits(id={id_}, rmin={rmin}, rmax={rmax}) 失败"
        self._write_raw_bytes(
            self._POS_LIMITS_START_ADDR,
            id_,
            data,
            num_retry=num_retry,
            raise_on_error=True,
            err_msg=err_msg,
        )

    def read_position_limits(self, motor: NameOrID) -> tuple[int, int]:
        """读取 Min_Position_Limit 与 Max_Position_Limit，返回 (rmin, rmax)。"""
        rmin = self.read("Min_Position_Limit", motor, normalize=False)
        rmax = self.read("Max_Position_Limit", motor, normalize=False)
        return (rmin, rmax)

    def wait_until_stopped(
        self,
        motor: NameOrID,
        timeout_s: float = 10.0,
        poll_interval_s: float = 0.05,
    ) -> bool:
        """轮询 Moving 寄存器直到为 0 或超时（与 STServo read_write 示例一致）。

        返回 True 表示已停止（Moving==0），False 表示超时或读失败。
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            try:
                moving = self.read("Moving", motor, normalize=False)
            except COMM_ERR:
                logger.debug("wait_until_stopped: 读 Moving 通信异常")
                time.sleep(poll_interval_s)
                continue
            if moving == 0:
                return True
            time.sleep(poll_interval_s)
        return False

    def write_pos_ex_and_wait(
        self,
        motor: NameOrID,
        position: int,
        speed: int,
        acc: int,
        timeout_s: float = 10.0,
        poll_interval_s: float = 0.05,
        *,
        num_retry: int = 0,
    ) -> bool:
        """一条指令写位置+速度+加速度，再轮询 Moving 直到停止（与 STServo read_write 示例一致）。

        先确保伺服模式（Operating_Mode=0），再写入 Goal_Position/Goal_Velocity/Acceleration，再 wait_until_stopped。
        返回 True 表示已到位，False 表示超时或写失败。
        """
        try:
            self.write("Operating_Mode", motor, 0)  # 确保伺服模式，否则 Goal_Position 不生效
            time.sleep(0.05)
            # 一条指令写入目标位置、速度、加速度（从寄存器 41 起连续 7 字节），与 STServo WritePosEx 一致
            id_ = self._get_motor_id(motor)
            pos_enc = self._encode_sign("Goal_Position", {id_: position})[id_]
            speed_enc = self._encode_sign("Goal_Velocity", {id_: speed})[id_]
            data = (
                [acc]
                + self._split_into_byte_chunks(pos_enc, 2)
                + [0, 0]
                + self._split_into_byte_chunks(speed_enc, 2)
            )
            self._write_raw_bytes(
                self._POS_EX_START_ADDR,
                id_,
                data,
                num_retry=num_retry,
                raise_on_error=True,
                err_msg=f"write_pos_ex_and_wait(id={id_}, pos={position}, speed={speed}, acc={acc}) 失败",
            )
            time.sleep(0.3)
        except COMM_ERR:
            return False
        result = self.wait_until_stopped(
            motor, timeout_s=timeout_s, poll_interval_s=poll_interval_s
        )
        time.sleep(0.1)
        return result

    def sync_write_pos_ex(
        self,
        values: dict[str, tuple[int, int, int]],
        *,
        num_retry: int = 0,
    ) -> None:
        """多舵机先 RegWritePosEx 写入缓冲，再 RegAction 统一执行（与 STServo reg_write 示例一致）。

        values: 电机名 -> (position, speed, acc)。所有电机使用同一 (position, speed, acc) 时可传多键同值。
        """
        

        for motor_name, (position, speed, acc) in values.items():
            id_ = self._get_motor_id(motor_name)
            model = self._get_motor_model(motor_name)
            pos_enc = self._encode_sign("Goal_Position", {id_: position})[id_]
            speed_enc = self._encode_sign("Goal_Velocity", {id_: speed})[id_]
            data = (
                [acc]
                + self._split_into_byte_chunks(pos_enc, 2)
                + [0, 0]
                + self._split_into_byte_chunks(speed_enc, 2)
            )
            for n_try in range(1 + num_retry):
                comm, error = self.packet_handler.regWriteTxRx(
                    self.port_handler, id_, self._POS_EX_START_ADDR, len(data), data
                )
                if self._is_comm_success(comm):
                    break
                logger.debug(
                    f"sync_write_pos_ex RegWrite id={id_}: {self.packet_handler.getTxRxResult(comm)}"
                )
            if self._is_error(error):
                logger.warning(
                    f"sync_write_pos_ex RegWrite id={id_}: {self.packet_handler.getRxPacketError(error)}"
                )
        comm = self.packet_handler.action(self.port_handler, scs.BROADCAST_ID)
        if not self._is_comm_success(comm):
            raise ConnectionError(
                f"sync_write_pos_ex RegAction 失败: {self.packet_handler.getTxRxResult(comm)}"
            )

    def _unfold_move_and_wait(
        self,
        motor: str,
        goal: int,
        timeout_s: float,
        tolerance_deg: float = UNFOLD_TOLERANCE_DEG,
    ) -> tuple[bool, int, str]:
        """伺服模式下用 WritePosEx 写目标，轮询 Moving 直到停止，再根据位置/Status 判断到达或堵转。误差在 tolerance_deg 度内视为成功（默认 5°）。"""
        goal = max(0, min(goal, FULL_TURN - 1))
        pos_now = self._read_with_retry("Present_Position", motor)
        print(f"    [{motor_label(motor)}] 当前位置={pos_now}, 目标位置={goal}")
        ok = self.write_pos_ex_and_wait(
            motor, goal, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION,
            timeout_s=timeout_s, poll_interval_s=0.05,
        )
        print(f"    [{motor_label(motor)}] 抬起角度结束: {ok}，开始判断位置")
        time.sleep(0.3)
        try:
            pos = self.read("Present_Position", motor, normalize=False)
        except COMM_ERR:
            self._clear_overload_unfold(motor)
            pos = self._read_with_retry("Present_Position", motor)
            err_deg = abs(pos - goal) * 360.0 / FULL_TURN
            print(f"    [{motor_label(motor)}] 堵转/异常停止: pos={pos} (距目标{abs(pos - goal)}步 ≈ {err_deg:.1f}°)")
            return False, pos, "堵转(通信异常)"
        if not ok:
            self._clear_overload_unfold(motor)
            err_deg = abs(pos - goal) * 360.0 / FULL_TURN
            print(f"    [{motor_label(motor)}] 超时: pos={pos} (距目标{abs(pos - goal)}步 ≈ {err_deg:.1f}°)")
            return False, pos, "超时"
        error_deg = abs(pos - goal) * 360.0 / FULL_TURN
        if error_deg <= tolerance_deg:
            print(f"    [{motor_label(motor)}] 到达目标: pos={pos} (误差{abs(pos - goal)}步 ≈ {error_deg:.1f}°，在{tolerance_deg}°内)")
            return True, pos, "到达"
        try:
            status = self.read("Status", motor, normalize=False)
        except COMM_ERR:
            status = 0
        if status & 0x20:
            self._clear_overload_unfold(motor)
            print(
                f"    [{motor_label(motor)}] 堵转停止: pos={pos} (Status=0x{status:02X} BIT5过载, "
                f"距目标{abs(pos - goal)}步 ≈ {error_deg:.1f}°)"
            )
            return False, pos, f"堵转(Status=0x{status:02X})"
        print(f"    [{motor_label(motor)}] 未到位停止: pos={pos} (距目标{abs(pos - goal)}步 ≈ {error_deg:.1f}°，超过{tolerance_deg}°)")
        return False, pos, "未到位"

    def _clear_overload_unfold(self, motor: str) -> None:
        """关扭矩清除 Overload 状态，等待恢复后重新上力矩（供展开逻辑使用）。"""
        try:
            self.write("Torque_Enable", motor, 0)
            time.sleep(UNFOLD_OVERLOAD_SETTLE + 0.1)
            self.write("Torque_Enable", motor, 1)
        except COMM_ERR:
            pass

    def unfold_single_joint(
        self,
        motor: str,
        unfold_angle: float,
        move_timeout: float,
    ) -> tuple[str | None, int]:
        """展开单个关节：先正方向，失败则反方向。

        初始化阶段已配好 PID/Acceleration/Operating_Mode=0，
        伺服模式下只需写 Goal_Position 即可驱动舵机。

        Returns:
            (成功方向, 步数): 方向为 "forward"/"reverse"，失败时为 (None, 0)；步数为目标步数。
        """
        target_steps = int(unfold_angle / 360.0 * FULL_TURN)
        print(f"\n--- 展开 {motor_label(motor)} ({target_steps}步 ≈ {unfold_angle:.1f}°) ---")

        # 校正中位（Torque_Enable=128 将当前位置设为 2048）
        self._write_torque_with_recovery(motor, 128)
        self._write_torque_with_recovery(motor, 1)

        time.sleep(0.1)
        print(
            f"[{motor_label(motor)}]设置当前位置为中位: Present_Position={self._read_with_retry('Present_Position', motor)}"
        )
        time.sleep(0.1)
        # 恢复伺服模式（128 可能改变了模式状态）并上力矩
        self.write("Operating_Mode", motor, 0)
        self._write_torque_with_recovery(motor, 1)
        time.sleep(0.3)
        # 尝试正方向
        # print(f"    [{motor_label(motor)}] 尝试正方向...")
        reached, pos_after, reason = self._unfold_move_and_wait(
            motor, MID_POS + target_steps, move_timeout
        )
        if reached:
            print(f"    [{motor_label(motor)}] 正方向成功")
            return "forward", target_steps

        moved_fwd = abs(pos_after - MID_POS)
        #print(f"    [{motor_label(motor)}] 正方向失败({reason})，移动 {moved_fwd} 步")

        # 回中位（堵转后 _clear_overload_unfold 已重新上力矩，带等待写中位）
        self.write_pos_ex_and_wait(
            motor, MID_POS, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION,
            timeout_s=5.0, poll_interval_s=0.05,
        )

        # 尝试反方向
        #print(f"    [{motor_label(motor)}] 尝试反方向...")
        reached, pos_after, reason = self._unfold_move_and_wait(
            motor, MID_POS - target_steps, move_timeout
        )
        if reached:
            print(f"    [{motor_label(motor)}] 反方向成功")
            return "reverse", target_steps
        moved_rev = abs(MID_POS - pos_after)
        #print(f"    [{motor_label(motor)}] 反方向也失败({reason})，移动 {moved_rev} 步，保持当前位置")
        print(f"    [{motor_label(motor)}] 展开失败，保持当前位置")
        return None, 0
