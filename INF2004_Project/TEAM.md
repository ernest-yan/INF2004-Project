# Team and subsystems

The tables are pre-filled with the **example** in `app_program/` (marked
*example*). Replace those rows with your own as you replace the example code.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi telemetry + remote commands over MQTT (no pins of its own) | 2502568 | caveman37 | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control: 2 DC motors (PWM) + 2 wheel encoders, speed/distance/turn goals, emergency brake | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line sensors + Code 39 barcode reader (left/centre IR line sensors, right IR barcode sensor) | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | GY-511 (LSM303DLHC) accelerometer: impact detection -> brake, hump-height profile | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | HC-SR04 ultrasonic + SG90 servo: obstacle stop request and obstacle profile scan | | | `app_program/subsystem5/` | `testing/subsystem5/` |
| — | *Team glue:* `vehicle_ctrl` task (connects S2/S3/S4/S5) | all | | `app_program/integration/` | `testing/integration/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S5 | HC-SR04 TRIG (do not use `CRASH_REPORT=1`) |
| GP1 | S5 | HC-SR04 ECHO (5 V output: use a divider to 3.3 V; do not use `CRASH_REPORT=1`) |
| GP2 | S3 | left line sensor (IR, input) |
| GP3 | | |
| GP4 | S3 | centre line sensor (IR, input) |
| GP5 | | |
| GP6 | S2 | left wheel encoder input |
| GP7 | S2 | right wheel encoder input |
| GP8 | S2 | left motor PWM A |
| GP9 | S2 | left motor PWM B |
| GP10 | S2 | right motor PWM A |
| GP11 | S2 | right motor PWM B |
| GP12 | S5 | servo PWM |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S3 | right barcode sensor (IR, input) |
| GP17 | | |
| GP18 | | |
| GP19 | | |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | S4 | I2C1 SDA (ADC0 capable) |
| GP27 | S4 | I2C1 SCL (ADC1 capable) |
| GP28 | | (ADC2 capable) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | S4 | `imu_task` (driver `subsystem4/drv_i2c1.c`; kernel I2C driver stays OFF) | GP26/GP27 |
| SPI0 | | | |
| SPI1 | | | |
| ADC | | | GP28 only (GP26/GP27 are I2C1: never set `DEVCONF_ADC_PIN_INIT_0/1`) |
| PWM slice 4, slice 5 | S2 | `motion_task` | GP8/GP9, GP10/GP11 |
| PWM slice 6 | S5 | `ultrasonic_task` | GP12 |
| IO_IRQ_BANK0 (GPIO interrupt, core 0) | S2 | `motion_task` (handler in `motion.c`; route any other GPIO IRQ through it) | GP6/GP7 rising edges |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | s1_telemetry_task – 13 – core 0 (core 1 is S3's barcode task); S1's MQTT code also runs inside the kernel's `cyw43_utk_task` (4) |
| 2 | 14–15 | motion_task – 14 – core 0 |
| 3 | 16–17 | barcode_task – 16 – core 1 (polls without sleeping); line_follow_task – 17 – core 0 |
| 4 | 18–19 | imu_task – 18 – core 0 |
| 5 | 20–21 | ultrasonic_task – 20 – core 0 |
| team | 11 | vehicle_ctrl – 11 – core 0 |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S3 → S3 | message buffer (owned by S3) | `mbf_barcode` | 1 byte per decoded Code 39 character ('A'–'Z'); `line_follow_task` receives with `tk_rcv_mbf` | |
| IF-02 | S5 → VC | message buffer (owned by S5) | `mbf_s5_events` | one `subsystem5_event_t` per STOP_REQUIRED / PROFILE_READY / FAULT; the VC must keep draining it | |
| IF-03 | VC, S3, S5 → S2 | function API (`motion.h`) | — | `b2_stop()`, `b2_submit_command()`, `b2_goal_start()`, `b2_goal_check()`, `b2_get_status()`; lock-protected, any task/core | |
| IF-04 | S4 → S2 | function call | — | `b2_emergency_brake()` on impact or IMU fault (latched until `b2_authorized_reset_while_stopped()`) | |
| IF-05 | S2 → S4 | function call | — | `imu_task` reads `b2_get_status()` odometry and feeds `b4_update_travel_mm()` | |
| IF-06 | VC → S5 | function call | — | `subsystem5_confirm_robot_stopped()` once the encoders are quiet for 100 ms | |
| IF-07 | S5 → S3 | function call | — | `subsystem5_get_status()`: S3 only drives while S5 is in MONITOR with no stop request | |
| IF-08 | S3 → S5 | function call | — | `get_line_barcode_data_safe()`: the bypass looks for the line on its way back | |
| IF-09 | S2 → S1 | function call | — | `b2_get_status()`: read-only, 5 Hz, from `s1_telemetry_task` only | |
| IF-10 | S3 → S1 | function call | — | `get_line_barcode_data_safe()`: read-only, 5 Hz, from `s1_telemetry_task` only (takes S3's mutex, so never from the WiFi task) | |
| IF-11 | S4 → S1 | function call | — | `b4_get_status()`: read-only, 5 Hz, from `s1_telemetry_task` only | |
| IF-12 | S5 → S1 | function call | — | `subsystem5_get_status()`: read-only, 5 Hz, from `s1_telemetry_task` only. S1 never reads `mbf_s5_events` (IF-02 belongs to the VC) | |
| IF-13 | S1 (WiFi task) → S1 (telemetry task) | message buffer (owned by S1) + `tk_wup_tsk` | `s1_mbf_cmd` | one `S1_CMD` per accepted remote command (START / STOP / E-STOP / RESET / MAP), max 4 queued; carried out by `s1_telemetry_task` (IF-15 to IF-18) | |
| IF-14 | S1 → kernel WiFi task | message buffer (owned by S1) | `s1_mbf_out` | telemetry frames (1 topic-ID byte + payload); emptied with `TMO_POL` by S1's hook in the shared `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | |
| IF-15 | S1 → S2 | function API (`motion.h`) | — | remote E-STOP → `b2_emergency_brake()` (latched); STOP → `b2_stop()`; RESET → `b2_authorized_reset_while_stopped()` (S2 refuses if an encoder ticked in the last 200 ms). From `s1_telemetry_task` only. **Proposed, awaiting team OK** | |
| IF-16 | S1 → S4, S1 → S5 | function call | — | remote RESET only, and only if the wheels are not being driven: `b4_authorized_reset_while_stopped()` if an impact or fault is latched (S4 then recalibrates), `subsystem5_reset_fault_while_stopped()` if S5 is in FAULT. **Proposed, awaiting team OK** | |
| IF-17 | S1 → S3 | function call | — | remote MAP (4 letters/digits: forward, U-turn, left, right) → `set_mqtt_action_mapping()`. Replaces the bench mapping; then S3 can set `S3_BENCH_MAPPING` to 0. **Format awaiting Buddy 3's OK** | |
| IF-18 | S1 → S3 | function call (`subsystem1.h`) | — | `s1_mission_go()`: TRUE after a remote START, FALSE at power-on and after STOP / E-STOP / RESET. **Proposed: S3's `line_follow_task` would check it before driving (not used yet)** | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 2 | 1 | 1 (+1 vehicle_ctrl) |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | | | | |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | 1 | | 1 |
| Message buffer | 12 | 2 | | 1 | | 1 |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | | | | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
