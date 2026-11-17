# Change Requirement 1: baseline ("before") measurements

- **Commit:** `18b1bd7` (git tag `pre-cr1`), 2026-11-17
- **Build:** `make NET=lwip MQTT=1` → `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.elf` (demo firmware)
- **Toolchain:** `arm-none-eabi-gcc`, `-O2`, Cortex-M0+
- **Measured:** 2026-11-18

## 1. Whole-image memory (`arm-none-eabi-size`)

| Section | Bytes | Meaning |
|---|---:|---|
| text | 369,349 | Code + constants (flash) |
| data | 96 | Initialised RAM |
| bss | 84,160 | Zero-initialised RAM |
| **RAM total (data + bss)** | **84,256** | Most of this is lwIP, CYW43 and kernel tables, which CR1 does not touch |

## 2. Application static RAM (app code only, test files excluded)

Sum of `.data` + `.bss` from each subsystem's object files. This is the number CR1 should reduce.

| Subsystem | Bytes |
|---|---:|
| S1 – Telemetry / MQTT | 409 |
| S2 – Motion | 263 |
| S3 – Line + barcode | 316 |
| S4 – IMU / terrain | 278 |
| S5 – Ultrasonic / servo | 323 |
| Integration (`vehicle_ctrl`) | 0 |
| **Total** | **1,589** |

The per-variable list is in [baseline_ram_symbols.txt](baseline_ram_symbols.txt) (file, symbol, bytes).

Task stacks are allocated by the kernel at run time and are not part of CR1:
`TASK_STACK_SIZE` = 2048 B per task, and the S1 telemetry task uses 4096 B.

## 3. In-RAM struct and message sizes (`sizeof`, ARM build)

### Inter-task messages (queues / mailboxes / message buffers)

| Type | Bytes | Used for |
|---|---:|---|
| `S1_CMD` | 12 | S1 command queue (`s1_mbf_cmd`) |
| `b2_command_t` | 16 | S2 motion command mailbox |
| `b2_event_t` | 2 | S2 → controller events |
| `b4_event_t` | 12 | S4 events |
| `subsystem5_event_t` | 32 | S5 event message buffer (`mbf_s5_events`) |
| `vehicle_event_t` | 8 | `qVehicleEvent` |
| barcode char | 1 | S3 `mbf_barcode` |

### Shared status / snapshot structs (task ↔ task, ISR ↔ task)

| Type | Bytes |
|---|---:|
| `b2_status_t` | 48 |
| `b2_config_t` | 18 |
| `b2_goal_t` | 24 |
| `LineBarcodeData_t` (already `packed`) | 6 |
| `b4_status_t` | 56 |
| `b4_terrain_t` | 56 |
| `b4_config_t` | 18 |
| `subsystem5_status_t` | 38 |
| `subsystem5_profile_t` | 28 |
| `subsystem5_sample_t` | 6 |
| `subsystem5_scan_samples_t` | 122 |
| `vehicle_status_t` | 4 |
| `motion_status_t` | 16 |
| `line_status_t` | 4 |
| `imu_status_t` | 18 |
| `ultra_status_t` | 14 |
| `S1_DIAG` | 12 |
| `S1_STATS` | 20 |

### S1 telemetry structs (unpacked form, before encoding)

| Type | Bytes |
|---|---:|
| `tlm_status_t` | 34 |
| `tlm_heartbeat_t` | 12 |
| `tlm_event_t` | 16 |
| `tlm_cmd_t` | 10 |

## 4. MQTT payload sizes on the wire

The S1 codec (`tlm_codec.h`) **already sends compact bit-packed binary, not text.** This part of CR1 is
met at the baseline.

| Frame | Bytes | Bits used |
|---|---:|---:|
| Online | 3 | 24 |
| Status (5 Hz) | 21 | 166 |
| Heartbeat | 11 | 86 |
| ACK | 7 | 52 |
| Event (max, obstacle profile) | 10 | 78 |
| Command | 5 | 40 |
| Command `map` | 9 | 72 |

## 5. Timing (not measured yet: needs the car)

Measure on hardware with the `pre-cr1` build before flashing the CR1 build:

- [ ] S2 control loop period and jitter (10 ms target)
- [ ] Encoder ISR execution time
- [ ] S3 line/barcode sample loop period
- [ ] S1 status frame rate (5 Hz target) and `deadline_misses`

## How to reproduce

```sh
git checkout pre-cr1
cd build_make && make NET=lwip MQTT=1 -j8
arm-none-eabi-size mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.elf
```
