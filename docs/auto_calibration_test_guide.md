# Auto-calibration Test Guide

This document describes how to run the **auto-calibration** for Feetech servos: a single command performs the full flow of unfolding to avoid interference, calibrating all servos, returning to mid, and folding.

---

## 1. Full flow overview

| Stage | Content | Description |
| --- | --- | --- |
| Stage 0 | Initialization | Stop all servos, Lock=1, configure PID/Acceleration/limits, enable torque |
| Stage 2 | Unfold servos 2-4 and record reference positions | Lift wrist_flex(4) → shoulder_lift(2) → elbow_flex(3) first; record unfold direction and reference position; then retract servos 2 and 3 |
| Calibrate 2, 3 | Multi-servo simultaneous calibration | shoulder_lift and elbow_flex measured/written back in one pass; servos 2 and 3 folded |
| Stage 3 | Calibrate servos 4, 5, 6 | After lifting the elbow 80° to avoid obstruction, calibrate wrist_roll, gripper, wrist_flex simultaneously; then fold servo 3, unfold servo 4, fold gripper |
| Stage 4 | Calibrate servo 1 and return to mid | Calibrate shoulder_pan, write back, return to mid |
| Results and release | Print results, fold, release | Print min/max/offset for each joint, fold the entire arm, press Enter to release torque |
| Persistence (optional) | Same scheme as manual calibration | Only with `--save`: write to servo EEPROM and save to `.../calibration/robots/so_follower/<robot_id>.json` (draccus format, loaded by the arm at startup) |

When using `--unfold-only`, only Stage 0 and Stage 2's unfold are executed (arm unfold only, no calibration), useful for debugging the unfold logic.

---

## 2. How to invoke

### 2.1 Environment setup

Install the package before using the CLI command:

```bash
pip install -e .
```

After installation, the `lerobot-measure-feetech-ranges` command is available. If you do not want to install, you can call Python directly:

```bash
python src/lerobot/scripts/lerobot_measure_feetech_ranges.py --port COM23
```

The examples below all use the Python invocation; substituting `lerobot-measure-feetech-ranges` has the same effect.

### 2.2 Basic usage

```bash
# Full flow (unfold + calibrate), no persistence
python src/lerobot/scripts/lerobot_measure_feetech_ranges.py --port COM23

# Full flow + persistence (EEPROM + same local file as manual calibration)
python src/lerobot/scripts/lerobot_measure_feetech_ranges.py --port COM23 --save

# Specify robot id; if matched with the arm's config.id, it loads automatically at startup
python src/lerobot/scripts/lerobot_measure_feetech_ranges.py --port COM23 --save --robot-id bimanual_follower_left

# Skip unfold (already unfolded or unfold not needed)
python src/lerobot/scripts/lerobot_measure_feetech_ranges.py --port COM23 --unfold-angle 0

# Calibrate only a single servo (skip unfold)
python src/lerobot/scripts/lerobot_measure_feetech_ranges.py --port COM23 --motor shoulder_pan

# Custom unfold angle
python src/lerobot/scripts/lerobot_measure_feetech_ranges.py --port COM23 --unfold-angle 30

# Debug arm unfold only (Stage 0 init + Stage 2 unfold; no calibration)
python src/lerobot/scripts/lerobot_measure_feetech_ranges.py --port COM23 --unfold-only
```

### 2.3 Servo names (IDs 1-6)

- `shoulder_pan` (1)
- `shoulder_lift` (2)
- `elbow_flex` (3)
- `wrist_flex` (4)
- `wrist_roll` (5)
- `gripper` (6)

### 2.4 Argument list

**Calibration parameters:**

| Argument | Meaning | Default |
| --- | --- | --- |
| `--port` | Serial port path (required) | none |
| `--motor` | Calibrate only this servo (skip unfold) | all |
| `--velocity-limit` | Calibration limit-probing speed (constant-speed mode Goal_Velocity) | 1000 |
| `--timeout` | Single-direction limit wait timeout (seconds) | 20.0 |

**Unfold parameters:**

| Argument | Meaning | Default |
| --- | --- | --- |
| `--unfold-only` | Run only the arm unfold (Stage 0 + Stage 2), no calibration; for debugging the unfold logic | off |
| `--unfold-angle` | Unfold angle (degrees); set to 0 to skip unfold | 45.0 |
| `--unfold-timeout` | Per-motion unfold timeout (seconds); arrival or stall ends early | 6.0 |

**Output parameters (same path and format as manual calibration):**

| Argument | Meaning | Default |
| --- | --- | --- |
| `--save` | Write to servo EEPROM and save to the same local path as manual calibration (draccus format) | not written |
| `--robot-id` | Robot id used in the saved filename; path is `.../calibration/robots/so_follower/<robot_id>.json`. Must match config.id used at arm startup. | default |

---

## 3. Detailed flow

### 3.1 Stage 0: Initialization

- First use `sync_write` to broadcast-clear residual Overload (no response packet needed)
- Reconnect and verify all servos
- Stop all 6 servos (`Torque_Enable=0`)
- Set `Lock=1` to lock EEPROM
- Configure registers (write then read back to verify; on mismatch, print set/read values):

| Register | Value | Description |
| --- | --- | --- |
| Return_Delay_Time | 0 | Reduce communication delay |
| Operating_Mode | 0 | Servo mode |
| Max_Torque_Limit | 1000 | Max torque |
| Torque_Limit | 600 | Operating torque limit |
| Acceleration | 50 | Acceleration (matches calibration_defaults) |
| P_Coefficient | 16 | Position-loop P coefficient (anti-jitter) |
| I_Coefficient | 0 | Position-loop I coefficient |
| D_Coefficient | 32 | Position-loop D coefficient |
| Min/Max_Position_Limit | 0 / 4095 | Temporary full range; written back after calibration |
| Homing_Offset | 0 | Temporarily zeroed |
| Torque_Enable | 1 | Enable torque |

(Special protections such as the gripper are set during the arm's `configure`; this script's Stage 0 does not set them separately.)

### 3.2 Stage 2: Unfold servos 2-4 and record reference positions

1. Lift in the order 4 → 2 → 3 (wrist_flex → shoulder_lift → elbow_flex): wrist_flex 80°, shoulder_lift 15°, elbow_flex 30°. Record each joint's unfold direction (forward/reverse) and reference position `(Present_Position + Homing_Offset) % 4096` for arc selection during calibration.
2. Fold shoulder_lift and elbow_flex back to mid.
3. Calibrate servos 2 and 3: run `measure_ranges_of_motion_multi` simultaneously for multiple servos (with the first rotation direction being opposite the lift direction); write back Homing_Offset and limits; servos 2 and 3 return to mid or are held, then fold servos 2 and 3.

When unfolding a single joint: re-zero midpoint (`Torque_Enable=128`), try forward/reverse `Goal_Position = 2048 ± target steps`; if a stall or timeout occurs, return to mid and try the other direction.

### 3.3 Stage 3: Calibrate servos 4, 5, 6

1. Lift the elbow 80° to avoid obstruction.
2. Calibrate wrist_roll(5), gripper(6), wrist_flex(4) simultaneously: one `measure_ranges_of_motion_multi`; after writing back, wrist_roll holds its current position and the others return to mid.
3. One `_fold_arm`: fold elbow_flex, unfold wrist_flex, fold gripper.

### 3.4 Stage 4: Calibrate servo 1 shoulder_pan and return to mid

1. Run multi-servo calibration (single-motor) for shoulder_pan; write back Homing_Offset and limits.
2. Return to mid (Goal_Position=mid).

### 3.5 Measurement and write-back (measure_ranges_of_motion_multi)

The core logic lives in `auto_calibration.py`: probe CW/CCW limits in constant-speed mode for each motor, use the reference position (if present) to pick the arc, compute the midpoint and Homing_Offset, then write back Homing_Offset and Min/Max position limits. After writing back, wrist_roll holds its current position and others return to mid. With multiple servos, a single call measures multiple motors at once.

### 3.6 Motion control strategy

In servo mode, **Goal_Velocity is not used to control speed**; instead the PID parameters and Acceleration control the motion behavior:

- `P_Coefficient=16`: a relatively low P gives a softer response and prevents jitter
- `Acceleration=50`: smooth acceleration (matches calibration_defaults)
- Just write `Goal_Position` to drive the servo

Constant-speed mode (used during calibration limit probing) still uses `Goal_Velocity` to control speed.

---

## 4. Stall detection

All stall detection uses **BIT5 of the Status register (address 65, 0x41)**:

- BIT5=1 indicates **overload (stall)**
- Calibration phase: only declares stall after BIT5=1 is detected 3 consecutive times (avoids false positives)
- Unfold phase: declares stall as soon as BIT5=1 is detected

Communication exceptions (RuntimeError/ConnectionError) are also treated as stall signals.

---

## 5. 0/4095 wrap-around handling

In single-turn mode the position wraps in [0, 4095]. The CW and CCW limits divide the circle into two arcs; the starting position determines which arc is the physically reachable range:

- Arc A = (pos_cw - pos_ccw) % 4096
- Arc B = (pos_ccw - pos_cw) % 4096
- Whichever arc the starting position lies on is the physical range
- Mid = arc start + range / 2

---

## 6. Registers involved

| Register | Address | Purpose |
| --- | --- | --- |
| Return_Delay_Time | 0x07 (7) | Response delay; 0 reduces comm delay |
| Min_Position_Limit | 0x09 (9) | Min angle limit |
| Max_Position_Limit | 0x0B (11) | Max angle limit |
| Max_Torque_Limit | 0x10 (16) | Max torque; init sets to 1000 |
| Phase | 0x12 (18) | BIT4: angle feedback mode, 0 = single turn (0-4095) |
| P_Coefficient | 0x15 (21) | Position-loop P coefficient; set to 16 (soft response) |
| D_Coefficient | 0x16 (22) | Position-loop D coefficient; set to 32 |
| I_Coefficient | 0x17 (23) | Position-loop I coefficient; set to 0 |
| Protection_Current | 0x1C (28) | Protection current (gripper set to 250 by arm configure) |
| Homing_Offset | 0x1F (31) | Position offset; BIT11 sign bit |
| Operating_Mode | 0x21 (33) | Operating mode; 0 = servo mode, 1 = constant-speed mode |
| Overload_Torque | 0x24 (36) | Overload protection torque (gripper set to 25 by arm configure) |
| Torque_Enable | 0x28 (40) | Torque switch; 128 = set current position to 2048 |
| Acceleration | 0x29 (41) | Acceleration; script Stage 0 sets to 50 (matches calibration_defaults) |
| Goal_Position | 0x2A (42) | Target position (servo mode; writing automatically enables torque) |
| Goal_Velocity | 0x2E (46) | Target velocity (used only in constant-speed mode for calibration) |
| Torque_Limit | 0x30 (48) | Torque limit; script Stage 0 sets to 600 |
| Lock | 0x37 (55) | EEPROM lock; 1 = locked |
| Present_Position | 0x38 (56) | Current position (read-only) |
| Status | 0x41 (65) | Servo status (read-only); BIT5=1 indicates overload/stall |

---

## 7. Expected results

### 7.1 On success

```
Serial port: COM23
Full calibration: ['wrist_roll', 'gripper', 'wrist_flex', 'elbow_flex', 'shoulder_lift', 'shoulder_pan']

==================== Stage 0: Initialization ====================
Initialized and torque enabled (P=16, Acc=50, Torque=600).

==================== Stage 2: Unfold joints 2-4 (45.0°) ====================
  ...

==================== Calibrating servos 2 and 3 (multi-servo, opposite of lift direction) ====================
  ...

==================== Stage 3: Calibrate servos 4-6 (multi-servo simultaneous) ====================
  ...

==================== Stage 4: Calibrate shoulder_pan (servo 1) and return to mid ====================
  ...

==================== Calibration results ====================
  shoulder_pan(1): min=..., max=..., mid=..., offset=...
  ...

Calibration complete. Press Enter to exit...

==================== Persistence (same scheme as manual calibration) ====================
Wrote calibration to servo EEPROM.
Wrote calibration to: .../calibration/robots/so_follower/<robot_id>.json
```

Exit code 0.

### 7.2 Anomalies

- Connect failure: prints error, exit code 1
- Measurement exception: prints error, exit code 1
- Ctrl+C interrupt: safely stops all servos, exit code 130

---

## 8. Implementation files and save path

| File | Description |
| --- | --- |
| `src/lerobot/scripts/lerobot_measure_feetech_ranges.py` | Main script (Stage 0 init, unfold, multi-servo calibration; persistence at the same path as manual calibration) |
| `src/lerobot/motors/feetech/auto_calibration.py` | Multi-servo measurement (`measure_ranges_of_motion_multi`), single-joint unfold, WritePosEx, stall detection, etc. |
| `src/lerobot/motors/feetech/feetech.py` | Bus and FeetechCalibrationMixin; exports `COMM_ERR` |
| `src/lerobot/motors/feetech/calibration_defaults.py` | Default parameters (speed, timeout, PID, Acceleration, joint order, etc.) |

**Persistence path (same as manual calibration)**: `HF_LEROBOT_CALIBRATION / "robots" / "so_follower" / "<robot_id>.json"`, which by default is `~/.cache/huggingface/lerobot/calibration/robots/so_follower/<robot_id>.json`. Uses draccus format (`dict[str, MotorCalibration]`); the arm loads the matching id at startup via `config.id`.
