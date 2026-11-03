# Integration review and fixes (round 2)

Pin plan used (agreed): **HC-SR04 on GP0 (TRIG) / GP1 (ECHO)**, **GY-511 on
I2C1 GP26 (SDA) / GP27 (SCL)**, line/barcode sensors unchanged on
**GP2 / GP4 / GP16**. Full map in `TEAM.md` section 2.

## Build status

The previous "FIXED" archive did **not compile**. This one builds cleanly
(no warnings from application code) with arm-none-eabi-gcc 14.2 and Pico SDK
2.2.0 in all of these configurations:

| Command | Image |
|---|---|
| `make -j8` (defaults) | `mtk3pico_smp1_usb_cdc_wifi.uf2` ← flash this one |
| `make TEST=1 -j8` | `mtk3pico_smp1_usb_cdc_wifi_test.uf2` |
| `make WIFI=none -j8` | `mtk3pico_smp1_usb_cdc.uf2` |
| `make SMP=0 -j8` | builds, but prints a warning (see below) |
| `make CONSOLE=uart WIFI=none -j8` | builds, warns that UART0 now uses GP0/GP1 |

It has **not been run on the robot yet**: everything below still needs the
bench tests listed in `testing/`.

## What was broken, and the fix

| # | Problem | Effect | Fix |
|---|---|---|---|
| 1 | S2 `subsystem2/motion.c` and S4 `subsystem4/imu.c` used Pico SDK headers/functions (`pico/stdlib.h`, `hardware/pwm.h`, `hardware/i2c.h`, `clock_get_hz`, `gpio_set_irq_enabled_with_callback`, …). This firmware does not link the SDK runtime. | Compile error. Even if forced to link, `clock_get_hz()` returns 0 without SDK start-up and the SDK IRQ code bypasses the kernel's vector table. | Ported to libbsp + direct registers: PWM via `pwm_set_*`; encoder interrupt via `tk_def_int(IO_IRQ_BANK0)` with a kernel handler; I2C via new polled driver `subsystem4/drv_i2c1.c` (register-for-register port of the SDK's blocking I2C). Shared helpers in `app_program/app_hw.h`. All changes marked `PORT:`. |
| 2 | `subsystem2_test.c` / `subsystem4_test.c` included public headers that did not exist. | `make TEST=1` failed. | Added both public headers (now `subsystem2/tkernel_adapter.h` and `subsystem4/tkernel_adapter.h`); replaced the placeholder tests with real ones (S2-U01…U06, S4-U01…U04). None of them spins the wheels. |
| 3 | The kernel never takes the **PWM block out of reset** (`USE_PTMR = 0`). | Motor PWM and S5's servo would never move. | `app_release_reset(RESETS_RESET_PWM)` in both S2 and S5 (release only, never re-reset, because they share the block). I2C1 is reset and released by S4's driver. |
| 4 | `<math.h>` / `<string.h>` clash with the kernel's own `size_t`, and the link has no `-lm`. | Compile/link error in S4. | `subsystem4/mathf.h`: small `sqrtf/atan2f/sinf/fabsf` (checked against libm: error < 0.0002°). `memset` replaced by struct copies. |
| 5 | Interrupt masking (`save_and_disable_interrupts`) only protects one core; S4's urgent brake and the controller can call S2 from a task while the S2 task or encoder IRQ is active. | Torn motion state on the dual-core build. | `app_lock()` = kernel `ISpinLock` (masks IRQs on this core **and** excludes the other core). Used in S2 and S4. |
| 6 | S2 and S4 configured their hardware in `subsystemN_init()` (inside `usermain`). | GPIO IRQ would be enabled on whichever core ran `usermain`; violates the "owning task" rule. | `b2_init()` now runs inside `motion_task`, `b4_init()` inside `imu_task` (both pinned to core 0). S4 retries every 1 s if the GY-511 does not answer, instead of staying dead. |
| 7 | Nobody read S5's event buffer or confirmed the robot was stopped. | On the first obstacle S5 would time out (STOP_TIMEOUT fault after 1.5 s), then the buffer fills and it latches EVENT_DELIVERY. | New team task `app_program/integration/vehicle_ctrl.c` (priority 11, core 0): stops S2 on STOP_REQUIRED / FAULT, confirms "stopped" to S5 after 100 ms with no encoder edges, logs the profile, drains `mbf_barcode`, prints a 5 s health line. |
| 8 | `b4_update_travel_mm()` was never called. | S4 hump detection could never complete. | `imu_task` feeds S2 odometry (mean of both wheels) every sample. |
| 9 | Unused `*_tkernel_adapter.c` files in S2, S4, S5 created duplicate, un-pinned tasks if anyone called them. | Confusing; risk of double init. | Removed in S5. In S2 and S4, `tkernel_adapter.c` was rewritten to hold the task glue (`subsystemN_init/start`, pinned owning task) and is the only copy. |
| 10 | `build_make/tools/elf2uf2` was a pre-built Linux binary that lost its execute bit in the zip. | `make` stops at "Permission denied" on every machine. | Removed; `make` now rebuilds it from `tools/elf2uf2/main.cpp` automatically. |

`subsystem3/` (line + barcode, GP2/GP4/GP16) was **not changed**.

## Things to check on the bench

1. **HC-SR04 ECHO is 5 V.** Put a divider on GP1 (e.g. 1 kΩ from ECHO to GP1,
   2 kΩ from GP1 to GND). The RP2040 is not 5 V tolerant.
2. **Never build with `CRASH_REPORT=1` or `CONSOLE=uart`** while the HC-SR04
   is plugged in: those builds put UART0 on GP0/GP1 (the build now warns).
3. **Use `SMP=1` (default).** The S3 barcode task polls without sleeping on
   core 1; on a single-core build it starves S4 and S5 (the build warns).
4. On boot the console should show, in this order:
   `[VC] microsecond timer OK`, `[S2] motion ready`,
   `[S4] LSM303DLHC ready ...`. If the timer line is a WARNING, S3 bar widths
   and S5 distances are wrong and the clock set-up needs looking at.
5. If S4 prints "not answering at 0x19", check SDA/SCL are not swapped and
   try address 0x18 (`B4_ACCEL_ADDR_DEFAULT` in `subsystem4/imu.h`).
6. Calibrate S2 (`B2_CAL_*` in `subsystem2/tkernel_adapter.c`: ticks/rev, wheel circumference,
   track) with the wheels lifted before any floor run.
7. HC-SR04 out-of-range returns a ~38 ms echo, longer than S5's 30 ms
   timeout, so in a large open space S5 counts it as an invalid reading and
   faults after 3 in a row (`FAULT_MONITOR_SENSOR`). Test in the real arena;
   if it trips, S5 should treat "no echo" as "clear" rather than "fault".
8. Encoder inputs use the pad's reset pull-down. If your encoder boards are
   open-collector, enable a pull-up for GP6/GP7.

## Still missing (mission logic: team decision, not a build fix)

`vehicle_ctrl` is a **safe** controller: it only ever stops the robot. Not
written yet:
- line following (S3 `line_detected`/raw bits -> `b2_submit_command(B2_CMD_SPEED)`),
- executing a barcode action (`get_action_for_character()` -> B2 turn goal),
- the obstacle bypass after S5's `PROFILE_READY`, then
  `subsystem5_ack_line_recovered()`,
- an operator reset path for a latched S2 brake / S4 impact.

These belong in `vehicle_ctrl.c`, which already has the inputs wired up.
