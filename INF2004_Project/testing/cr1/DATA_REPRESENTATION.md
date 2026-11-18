# Change Requirement 1: data representation table

**Status:** v3, implemented 2026-11-18 on branch `cr1/min-bit`. Decisions D1–D6 approved. Checked against the Project Write-up, the Project Briefing and the IMU change notice. The "After" columns below were planning estimates; the **measured** sizes are in [CR1_REPORT.md](CR1_REPORT.md), and the differences from the plan are listed in "As implemented" at the end of this file.

This table lists each piece of application data in S1–S5, its valid range, the minimum bits it needs, and how it will be stored.
"Before" sizes come from [BASELINE.md](BASELINE.md). "After" sizes are estimates for now; step 5 replaces them with measured values.

## Rules used throughout

| # | Rule | Why |
|---|---|---|
| R1 | States and enum fields become bit-fields sized to their value count (`uint8_t state : 3`). | `arm-none-eabi` stores each enum in a full byte (8 bits) even when it holds 5 values. |
| R2 | Each fault becomes **one bit in a fault bitmask**, not a single "current fault" enum. | Required by the change requirement. Several faults can also be active at once. |
| R3 | `bool`/`BOOL` flags become 1-bit fields grouped in one flags byte per module. | `BOOL` is 4 bytes and `bool` is 1 byte; a flag needs 1 bit. |
| R4 | Counters used only as event triggers or wire values shrink to the wire width and wrap (e.g. `uint16_t` → `uint8_t`). | The dashboard only checks whether a counter changed. |
| R5 | Timestamps go from `uint64_t` µs to `uint32_t` µs, compared with wrap-safe subtraction `(now - t) < limit`. | 32 bits covers 71.6 minutes, longer than any run. |
| R6 | **Exception:** data written by the encoder ISR keeps full aligned 32-bit words. | On the Cortex-M0+, an aligned 32-bit load/store is a single instruction, so the ISR never needs a read-modify-write on packed data. |
| R7 | **Exception:** floating-point working variables inside the S4 filter and hump algorithms stay `float`. | They are intermediate maths, not stored or exchanged application data. Converting them would change tuned behaviour that we can't re-test without the car. |
| R8 | The MQTT wire format is **unchanged**: it is already bit-packed (see S1). | A change would mean re-testing the dashboard as well. |
| R9 | Fixed calibration settings become `const` (flash), not RAM copies. | Several configs are copied twice into RAM (`s2_config` + `s_cfg`). |
| R10 | Bit-fields use **unsigned** types only. Signed values go in offset-binary form (stored = value + offset), the same way the MQTT codec already sends pitch and yaw. | The brief requires the Barr C coding standard, which forbids bit-fields in signed integer types. |

## Concurrency rules for packed shared data

| Shared data | Writers → readers | Protection |
|---|---|---|
| S2 encoder counts / last edge time | Encoder ISR (core 0) → S2 task (core 0) | Unpacked 32-bit words (R6). Task reads them inside `app_lock()`, which masks the IRQ and takes a spinlock. **Already correct at the baseline.** |
| S2 `s_last_edge_us` (`uint64_t`) | ISR → task | A 64-bit write is **two** stores on M0+. Readers already use `app_lock()`. R5 makes it `uint32_t`, a single store. |
| S2 status / command mailbox | S2 task ↔ VC, S1, S3, S5 tasks (either core) | Existing `app_lock()` around every packed read-modify-write. |
| S3 `LineBarcodeData_t` | S3 task → S1, VC | Existing kernel mutex `g_mtx`, accessed via `get_line_barcode_data_safe()`. |
| S3 barcode map (4 letters) | S1 task → S3 task | Packed into **one `uint32_t`** and written with a single store, so S3 can never see half a map. This is safer than the baseline's four separate `char` writes. |
| S4 status | S4 task → S1, VC | Existing `s_lock` (`app_lock`). |
| S5 status / snapshot | S5 task → S1, VC | Existing mutex `s_mtx` + snapshot copy. |
| **New:** system fault mask | Every subsystem → S1, VC | `uint16_t`, set/cleared only via `app_fault_set()` / `app_fault_clear()` under `app_lock()`. M0+ has no atomic OR instruction (no LDREX/STREX), so a plain `mask \|= bit` would be a race. |

## System fault bitmask (new, R2)

A single `uint16_t app_fault_mask` in `app.h` replaces the per-subsystem fault enums as the shared view of faults. Each subsystem's own enum bits map into it.

| Bit | Name | Source | Replaces |
|---:|---|---|---|
| 0 | `FLT_MOTOR_STALL` | S2 | `B2_FAULT_ENCODER_STALL` |
| 1 | `FLT_GOAL_TIMEOUT` | S2 | `B2_FAULT_GOAL_TIMEOUT` |
| 2 | `FLT_S2_CONFIG` | S2 | `B2_FAULT_BAD_CONFIG` |
| 3 | `FLT_S2_EVENT_LOSS` | S2 | `B2_FAULT_EVENT_LOSS` |
| 4 | `FLT_LINE_LOSS` | S3 | `FAULT_LINE_LOSS` |
| 5 | `FLT_IMU_I2C` | S4 | `B4_FAULT_I2C` |
| 6 | `FLT_IMU_CONFIG` | S4 | `B4_FAULT_BAD_CONFIG` |
| 7 | `FLT_IMU_QUEUE` | S4 | `B4_FAULT_QUEUE_FULL` |
| 8 | `FLT_NO_ECHO` | S5 | `subsystem5_FAULT_MONITOR_SENSOR` |
| 9 | `FLT_SCAN_INVALID` | S5 | `subsystem5_FAULT_SCAN_INVALID` |
| 10 | `FLT_STOP_TIMEOUT` | S5 | `subsystem5_FAULT_STOP_TIMEOUT` |
| 11 | `FLT_S5_EVENT` | S5 | `subsystem5_FAULT_EVENT_DELIVERY` |
| 12 | `FLT_NO_SAFE_SIDE` | S5 | `subsystem5_FAULT_NO_SAFE_SIDE` |
| 13 | `FLT_BYPASS` | S5 | `subsystem5_FAULT_BYPASS` |
| 14 | `FLT_WIFI_LOSS` | S1 | link state ≠ connected |
| 15 | spare | | |

The wire keeps the existing 8-bit per-subsystem `fault_mask` (bit n = subsystem n faulted), derived from this mask. See decision D5.

---

## S1 – Telemetry / MQTT

**MQTT payloads already meet the requirement** (binary, MSB-first bit packing in `tlm_codec.c`). The wire layout is listed here for the table.

### Status frame (5 Hz), 166 bits = 21 bytes

| Field | Valid range | Bits | Representation |
|---|---|---:|---|
| session | 0..65535 | 16 | uint |
| seq | 0..255, wraps | 8 | uint |
| vehicle_state | 7 states | 3 | enum code |
| b_estop_latched | 0/1 | 1 | flag |
| line_state | 4 states | 2 | `(left<<1)\|centre` |
| last_barcode | 6 values | 3 | enum code |
| speed_l / speed_r | 0..1023 mm/s, saturates | 10 + 10 | uint |
| enc_l / enc_r | 16-bit, wraps | 16 + 16 | uint |
| distance | 0..4095 cm, saturates | 12 | uint |
| pitch | −64..63 ° | 7 | value + 64 |
| hump_max | 0..255 mm | 8 | uint |
| obstacle_state | 8 states | 3 | enum code |
| obstacle distance | 0..511 cm, saturates | 9 | uint |
| fault_mask | bits S2..S5 | 8 | bitmask |
| yaw rate | −256..255 °/s | 9 | value + 256 |
| motion | 7 classes | 3 | enum code |
| up speed | 0..1020 mm/s | 8 | value / 4 |
| tilt | 0..127 °, saturates | 7 | uint |
| s5_fault | 7 values | 3 | enum code |

Other frames: online 24 bits (3 B), heartbeat 86 bits (11 B), ACK 52 bits (7 B), event ≤ 78 bits (10 B), command 40 bits (5 B), `map` command 72 bits (9 B). Field-by-field layouts are in `tlm_codec.h`.

### S1 RAM data

| Field / struct | Before | Valid range | Bits | After | Notes |
|---|---|---|---:|---|---|
| `S1_CMD.code` | `UB` 8 | 6 commands | 3 | bit-field | |
| `S1_CMD.map[4]` | 4 × `char` | 'A'–'Z', '0'–'9' (36) | 4 × 6 | 24-bit field | code + map share one `uint32_t` |
| `S1_CMD.seq` | `UH` 16 | 0..65535 | 16 | unchanged | |
| `S1_CMD.t_rx_us` | `UW` 32 | µs, wraps | 32 | unchanged | |
| **`S1_CMD` (queue message)** | **12 B** | | | **≈ 10 B** | the kernel rounds the slot up to 4 B |
| `S1_STATS` counters ×4 | `UW` ×4 | wire = 8 bits, wraps | 8 each | `uint8_t` ×4 | `last_cmd_us` stays 32-bit |
| **`S1_STATS`** | **20 B** | | | **≈ 8 B** | |
| `g_rx` MQTT receive buffer | 289 B | raw bytes | | unchanged | byte buffer, nothing to pack |
| `tlm_status_t` staging struct | 34 B | as wire | | bit-field struct ≈ 22 B | optional, low value |

### Team interface structs (`team_interfaces.h`)

| Struct | Before | Packed fields | After |
|---|---:|---|---:|
| `vehicle_status_t` | 4 B | state 3, estop 1, source 3, fault_mask 8 | 2 B |
| `vehicle_event_t` (queue `qVehicleEvent`) | 8 B | type 2, source 3, arg 16, t_rx 32 | 8 B (already minimal once aligned) |
| `motion_status_t` | 16 B | speeds 10+10, enc 16+16, distance cm 12 | 8 B |
| `line_status_t` | 4 B | line 2, barcode 3, count 8 (R4) | 2 B |
| `imu_status_t` | 18 B | pitch 7, hump 8+8, counts 8+8, cause 2, yaw 9, motion 3, up 8, tilt 7 | ≈ 10 B |
| `ultra_status_t` | 14 B | state 3, fault 3, nearest 9, count 8, centre 8, width 8, clear 9+9, decision 2 | ≈ 9 B |

---

## S2 – Motion

| Field | Before | Valid range | Bits | After | Notes |
|---|---|---|---:|---|---|
| `b2_status_t.state` | enum 8 | 5 states | 3 | bit-field | R1 |
| `b2_status_t.fault` | enum 8 | 4 faults | 4 | bitmask | R2 |
| `brake_latched` | `uint8_t` | 0/1 | 1 | bit-field | R3 |
| `left/right_encoder_edges` | `uint32_t` ×2 | 0..2³²−1, wraps | 32 | unchanged | copy of the ISR counter (R6) |
| `left/right_total_mm` | `int32_t` ×2 | ±32.7 m **[confirm D1]** | 16 | `int16_t`, saturates | |
| `left/right_speed_mm_s` | `int32_t` ×2 | ±511 mm/s (max is 300) | 10 | unsigned bit-field, value + 512 | R10 |
| `left/right_pwm_permil` | `uint16_t` ×2 | 0..1000 | 10 | bit-field | |
| `deadline_misses` | `uint32_t` | count, saturates | 16 | `uint16_t` | |
| `goals_done` | `uint32_t` | count, wraps | 8 | `uint8_t` | goals only compare it for change |
| `last_update_us` | `uint64_t` | µs | 32 | `uint32_t` | R5 |
| **`b2_status_t`** | **48 B** | | | **≈ 28 B** | |
| `b2_command_t.type` | enum 8 | 5 types | 3 | bit-field | |
| `left/right_mm_s` | `int16_t` ×2 | ±511 mm/s | 10 | unsigned bit-field, value + 512 | R10 |
| `distance_mm` | `uint16_t` | 0..8191 mm **[confirm D1]** | 13 | bit-field | |
| `turn_degrees` | `uint16_t` | 0..360 ° | 9 | bit-field | |
| `goal_speed_mm_s` | `uint16_t` | 0..511 mm/s | 9 | bit-field | |
| `timeout_ms` | `uint32_t` | 1..65535 ms (S3 uses 6000) | 16 | `uint16_t` | |
| **`b2_command_t` (mailbox)** | **16 B** | | | **≈ 10 B** | |
| `b2_event_t` (type, fault) | 2 B | 4 types, 4 faults | 2 + 4 | one byte | **1 B** |
| `b2_goal_t` | 24 B | goals_done 8, started_us 32, seen 1 | | | **≈ 8 B** |
| `s_irq_count[2]` | `volatile uint32_t` | wraps | 32 | unchanged | ISR (R6) |
| `s_last_edge_us[2]` | `volatile uint64_t` | µs | 32 | `uint32_t` | R5, removes a non-atomic 64-bit store |
| `s_signed_encoder_ticks[2]` | `int64_t` | ±2³¹ ticks ≈ ±474 km | 32 | `int32_t` | |
| `s_wheel_started_us`, `s_last_poll_us`, `s_goal_started_us`, `s_last_stopped_check_us` | `uint64_t` | µs | 32 | `uint32_t` | R5 |
| `s_integral[2]` | `int32_t` | PI accumulator | 32 | unchanged | maths state, already 32-bit |
| `s_ready`, `s_pending`, `s_*_event_pending` (5) | `bool` ×5 | 0/1 | 5 | one flags byte | R3, under `s_lock` |
| `s_cfg` + `s2_config` | 18 B + 18 B | calibration | | `const` | R9 |

---

## S3 – Line + barcode

| Field | Before | Valid range | Bits | After | Notes |
|---|---|---|---:|---|---|
| `LineBarcodeData_t` IR + flags | bit-fields | 5 flags | 5 | unchanged | **already compact at baseline** |
| `scanned_char`, `last_char` | `char` ×2 | Code 39: 43 symbols + none | 6 each | 6-bit codes | |
| `fault_mask` | `uint8_t` | 1 fault | 1 | moved into the spare bits | |
| `barcode_count` | `uint16_t` | wraps | 8 | `uint8_t` | R4 |
| **`LineBarcodeData_t`** | **6 B** | | | **4 B** | |
| `g_widths[60]` bar widths | `uint32_t` µs ×60 = 240 B | 0..400 000 µs (timeout) | 16 | `uint16_t` in 8 µs units (max 524 ms) **[confirm D2]** | **saves 120 B**, the largest single saving |
| `g_mqtt_map_*` | `char` ×4 | 36 symbols each | 4 × 6 | one `uint32_t` | single-store update (see concurrency) |
| `MovementAction_t` | enum 8 | 5 actions | 3 | bit-field | |
| `g_edge_pending`, `g_have_successful_decode`, `g_scan_active`, `g_sensor_state_initialized` | `bool` ×4 | 0/1 | 4 | flags byte | R3 |
| `g_last_sensor_state` | `uint8_t` | 3 sensors | 3 | in the same flags byte | |
| `g_width_count` | `uint8_t` | 0..60 | 6 | bit-field | |
| function statics `searching`, `turning`, `exiting`, `last_go_left`, `s_lf_following` | `BOOL` 4 B ×5 | 0/1 | 5 | flags byte | R3 |
| line-follow state (implied by flags) | flags | follow / search / turn / exit / idle | 3 | one 3-bit state field | clearer than 5 separate flags |
| timestamps (`*_since`, `*_time`) | `uint32_t` | µs, wraps | 32 | unchanged | already 32-bit |
| `mbf_barcode` message | 1 B char | 'A'–'Z' | 6 | unchanged (1 B) | the smallest message unit the kernel supports |

---

## S4 – IMU / terrain

Per the IMU change notice, S4 detects **lifted / overturned** (not collisions) and brakes the motors through `b2_emergency_brake()`, latched until RESET. The code already does this (`imu.c`, `posture.c`). There are no collision or impact fields to represent.

| Field | Before | Valid range | Bits | After | Notes |
|---|---|---|---:|---|---|
| `b4_status_t.state` | enum 8 | 4 states | 2 | bit-field | |
| `fault` | enum 8 | 3 faults | 3 | bitmask | R2 |
| `calibration_progress` | `uint8_t` | 0..100 % | 7 | bit-field | |
| `calibration_ready`, `lift_latched` | `uint8_t` ×2 | 0/1 | 2 | flags | |
| `acceleration_x/y/z_mg` | `int16_t` ×3 | ±2048 mg (±2 g range) | 12 each | unchanged **[confirm D3]** | 3 × 12 = 36 bits; packing saves 1 B and costs shifts on every sample |
| `pitch_cdeg` | `int16_t` | ±18000 | 16 | unchanged | already minimal |
| `tilt_cdeg`, `peak_tilt_cdeg` | `uint16_t` | 0..18000 | 15 | unchanged | already minimal |
| `peak/recent_up_speed_mm_s` | `int16_t` | ±1023 mm/s | 11 | unchanged | |
| `max/last_hump_height_mm`, `current_peak_mm` | `uint16_t` ×3 | 0..255 mm | 8 | `uint8_t` ×3 | matches the wire |
| `hump_count` | `uint16_t` | wraps | 8 | `uint8_t` | R4 |
| `terrain_phase` | enum 8 | 4 phases | 2 | bit-field | |
| `lift_cause` | enum 8 | 3 causes | 2 | bit-field | |
| `yaw_rate_dps` | `int16_t` | −256..255 °/s | 9 | unsigned bit-field, value + 256 | R10, same as the wire |
| `motion` | enum 8 | 7 classes | 3 | bit-field | |
| `i2c_errors` | `uint32_t` | count, saturates | 16 | `uint16_t` | |
| `calibration_restarts` | `uint32_t` | count, saturates | 8 | `uint8_t` | |
| `last_sample_us` | `uint64_t` | µs | 32 | `uint32_t` | R5 |
| **`b4_status_t`** | **56 B** | | | **≈ 34 B** | |
| `b4_event_t` | 12 B | type 2, height 8, pitch 16+16, gap 0..1023 → 10, cause 2, fault 3 | | | **≈ 8 B** |
| `b4_terrain_t` floats | `float` ×3 | algorithm state | | unchanged | R7 |
| `b4_terrain_t` path counters | `uint32_t` ×6 | 0..65 m per hump | 16 | `uint16_t` ×6 | |
| `b4_terrain_t` sample counters | `uint16_t` ×2 | 0..255 samples | 8 | `uint8_t` ×2 | |
| **`b4_terrain_t`** | **56 B** | | | **≈ 40 B** | |
| `b4_posture_t` floats | `float` ×10 | filter state | | unchanged | R7 |
| `b4_posture_t` hold counters | `uint8_t` ×5 | 0..100 | 7 | unchanged | already a byte each |
| `s_cfg` + `s4_config` | 18 B + 18 B | calibration | | `const` | R9 |

---

## S5 – Ultrasonic / servo

| Field | Before | Valid range | Bits | After | Notes |
|---|---|---|---:|---|---|
| `subsystem5_status_t.state` | enum 8 | 8 states | 3 | bit-field | |
| `fault` | enum 8 | 6 faults | 6 | bitmask | R2 |
| `front_mm` | `uint16_t` | 0..4000 mm (HC-SR04) + "unknown" | 12 | bit-field, 4095 = unknown | |
| `front_valid`, `stop_required`, `scan_complete` | `uint8_t` ×3 | 0/1 | 3 | flags | |
| `profile_count` | `uint16_t` | wraps | 8 | `uint8_t` | R4 |
| `subsystem5_sample_t` | 6 B | angle 0..180 (8), distance 12, valid 1 | 21 | 4 B (`uint32_t` bit-fields) | |
| `subsystem5_scan_samples_t` (5 + 15 samples) | 122 B | 20 × 4 + counts 3 + 4 bits | | **≈ 82 B** | saves 40 B |
| `profile` flags (5) | `uint8_t` ×5 | 0/1 | 5 | flags byte | |
| `profile` distances (closest, width, clearances, edge mm) | `uint16_t` ×6 | 0..4000 mm | 12 each | bit-fields | |
| `profile` angles (closest, span, edges) | `uint16_t` ×4 | 0..180 ° | 8 each | `uint8_t` | |
| `candidate_side` | enum 8 | 3 sides | 2 | bit-field | |
| **`subsystem5_profile_t`** | **28 B** | | | **≈ 16 B** | |
| **`subsystem5_status_t`** (+ `s_snapshot` copy) | **38 B** ×2 | | | **≈ 22 B** ×2 | |
| **`subsystem5_event_t`** (message buffer) | **32 B** | type 2, fault 6, front 12, profile | | **≈ 20 B** | |
| `s_bp_phase`, `s_echo_phase` | `uint8_t` | small state machines (≤ 8) | 3 each | bit-fields | |
| `s_bp_dir` | `uint8_t` | left/right | 1 | bit-field | |
| 1-bit flags (`s_bp_seen`, `s_echo_disturbed`, `s_disturbed_run`, `s_range_active`, `s_monitor_invalid`, `s_fault_reset_request`, `s_line_recovered_ack`, `s_stationary_scan_requested`, `s_stopped_confirmed`) | `uint8_t` ×9 | 0/1 | 9 | 2 flags bytes | R3 |
| small counters (`s_close_count`, `s_bp_clear_count`, `s_reverse_count`, `s_scan_retries`, `s_scan_index`) | `uint8_t` ×5 | ≤ 20 | 3..5 each | unchanged | already a byte; packing would cost more code than it saves |
| `s_servo_angle` | `uint8_t` | 0..180 | 8 | unchanged | already minimal |
| `s_bypass_goal` | 24 B | goal timing | | ≈ 8 B | same as `b2_goal_t` |

---

## Expected totals (planning estimates; measured values in CR1_REPORT.md)

| | Before | After (est.) |
|---|---:|---:|
| App static RAM | 1,589 B | ≈ 1,150 B (−25 %) |
| S2 command message | 16 B | ≈ 10 B |
| S5 event message | 32 B | ≈ 20 B |
| S1 command message | 12 B | ≈ 10 B |
| S2 event message | 2 B | 1 B |
| MQTT status frame | 21 B | 21 B (already minimal; a JSON equivalent is far larger, see D6) |

## Decisions needed

| # | Question | Proposal |
|---|---|---|
| D1 | Longest distance the car can drive in one run or in one command? | ±32.7 m per run (`int16_t` mm) and ≤ 8.19 m per command (13 bits). The demo course is far shorter. |
| D2 | Bar-width resolution for the barcode decoder? | 8 µs units in `uint16_t`. At 70 mm/s a 1 mm narrow bar is ~14 000 µs, so 8 µs costs nothing. |
| D3 | Pack the IMU accelerometer axes into 12-bit fields? | **No.** It saves 1 byte and adds cost to every 100 Hz sample. |
| D4 | Keep `float` inside the S4 filters (R7)? | **Yes.** They can't be re-tuned without the car. |
| D5 | Send the full 16-bit fault mask over MQTT? | **No, for now.** It would grow the status frame from 21 to 22 B and need a dashboard change. The 8-bit per-subsystem mask stays. |
| D6 | Use `tlm_format_status_json()` (already in the code) to report the text-vs-binary comparison? | **Yes.** It gives the "text payload" size for the message size comparison at no cost. |

---

## Requirements cross-check

Packing must not drop or narrow any data the specification asks for.

| Requirement (source) | Data | Still covered? |
|---|---|---|
| Telemetry: robot state, motor speed, encoder counts, line sensor status, barcode, hump measurements, obstacle scan results, distance travelled (Write-up, Buddy 1) | MQTT status and event frames | Yes: wire format unchanged (R8) |
| Encoder-based motion: `moveForward(distance)`, `turnLeft(angle)`, `stop()` … (Write-up, Buddy 2) | `b2_command_t` | Yes: distance ≤ 8.19 m, angle 0..360 ° (D1) |
| Barcode commands Turn Left / Right / Straight / U-Turn, example letters A–D (Write-up) | `MovementAction_t`, map letters | Yes: 5 actions in 3 bits; letters as 6-bit codes |
| Detect humps and report the highest hump peak (Write-up 4, 5) | `max_hump_height_mm` | Yes: 0..255 mm (the course hump is ~10 mm) |
| Tilt detection, motion classification, turn-rate monitoring (Write-up, Buddy 4) | pitch, tilt, motion, yaw | Yes: full ranges kept |
| **IMU change notice:** detect lifted / overturned and stop all motors | `lift_cause`, `lift_latched`, S2 brake | Yes: 2-bit cause + 1-bit latch |
| Obstacle location, width, closest point, left/right clearance (Write-up, Buddy 5) | `subsystem5_profile_t` | Yes: 0..4000 mm, 0..180 ° |
| Avoidance plan: stop, continue, turn left/right, reverse and reattempt (Write-up, Buddy 5) | S5 state + candidate side | Yes: 8 states (3 bits), 3 sides (2 bits) |
| C, micro T-Kernel, Barr C coding standard (Write-up overview) | all | Yes: R10 |
| Resource efficiency (Write-up, assessment) | all | This change requirement |

---

## As implemented: differences from the plan

| Item | Plan | Implemented | Why |
|---|---|---|---|
| App static RAM | ≈ 1,150 B | **1,122 B** (measured) | |
| `b2_command_t` | packed, ≈ 10 B | Caller's request keeps plain types (14 B, on the caller's stack). The **shared mailbox** is packed to **8 B**. | A request is range-checked before packing, so an out-of-range value (e.g. 10,000 mm/s) is refused instead of silently truncated by a bit-field. |
| S2 goal timeout | `uint16_t` ms | 10-bit field in 10 ms units (≤ 10.23 s, rounded up) | Fits the 8-byte mailbox. The longest timeout in use is 6 s. |
| S2 PI integral | `int32_t` | `int16_t` | Clamped to ±8000 by the controller. |
| `S1_CMD` | ≈ 10 B | Unchanged, 12 B | 75 bits of data need 10 B; 4-byte alignment and the kernel's 4-byte message slots make it 12/16 B either way. |
| Barcode letters | 6-bit Code 39 symbol | 6-bit code: 0 = none, 1–26 = A–Z, 27–36 = 0–9 | The decoder only outputs A–Z, and the same code holds the MQTT map letters (A–Z, 0–9). |
| `b4_status_t.yaw_rate_dps` | 9-bit offset field | `int16_t` | Written on every 100 Hz sample outside the lock; a packed field would need the lock on every sample. |
| `b4_status_t.calibration_progress` | 7-bit field | `uint8_t` | Same reason (100 Hz path). |
| `b4_terrain_t` | ≈ 40 B | 40 B | Floats unchanged (D4). |
| S4 control fields | bit-fields | bit-fields, **every write under `s_lock`** | The remote RESET writes them from S1's task. |
| `subsystem5_status_t` fault | bitmask | bitmask, plus `subsystem5_first_fault()` for the unchanged 3-bit MQTT field | D5: wire format unchanged. |
| S5 command flags (`s_stopped_confirmed`, …) | flags byte | one byte each (unchanged) | Written by other tasks; a byte store needs no lock, a shared bit-field byte would. Saves only 3 B. |
| S5 small counters | unchanged | unchanged | As planned. |
| System fault bitmask | `uint16_t app_fault_mask` with set/clear under a lock | `system_fault_mask()` builds the same 16 bits on demand from each subsystem's fault bitmask | No second copy of the faults to keep in step, and no writer, so no lock. The bit layout is the table above (`SYS_FLT_*` in `team_interfaces.h`). |
| `vehicle_event_t` | 8 B | 8 B | 53 bits of data; 8 B once aligned. |

