# Subsystem 2: motion calibration run

This run measures the three numbers that every distance and turn depends
on, and records a speed step response for tuning. The firmware drives the
robot and prints what the encoders saw. You measure what the robot
actually did, then put the corrected values in
[`app_program/subsystem2/motion_cal.h`](../../app_program/subsystem2/motion_cal.h).

That header is the only place the calibration lives. S2 builds its
configuration from it, and S5 derives its bypass tick scales from it, so
one change reaches both.

| Value in `motion_cal.h` | Now | Measured by |
|---|---|---|
| `B2_CAL_TICKS_PER_REV_LEFT` / `_RIGHT` | 930 / 930 | S2-C01 (hand turns) |
| `B2_CAL_WHEEL_CIRCUMFERENCE_MM` | 210 (provisional) | S2-C02 (drive 1 m) |
| `B2_CAL_WHEEL_TRACK_MM` | 120 (provisional) | S2-C03, S2-C04 (turn 360°) |

## What you need

- About 2 m × 1 m of plain floor with **no black line** and nothing in front of the ultrasonic sensor.
- A tape measure, masking tape and a pen.
- A laptop with a serial terminal on the USB console, **logging to a file** so the `S2-STEP` lines can be kept.

## Build and flash

```sh
cd INF2004_Project/build_make
make TEST=1 NET=lwip MQTT=1 S2_CAL=1 -j8
```

Flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test_s2cal.uf2`.

The test build keeps S3 (line following) and S5 (bypass) from driving, so
only S2's calibration moves the wheels. S5 still stops the robot if
something is close in front of it, so keep the path clear.

## Steps

Power the robot on **standing still on the floor**, so the IMU can
calibrate. The S1–S2 self-tests run first. Then the console shows
`[S2-CAL] ===== calibration run` and leads you through the steps below
with 5-second countdowns.

| Step | You do | Console prints | Then |
|---|---|---|---|
| — | Lift the robot so both wheels turn freely (10 s) | `lift now` countdown | Lifting it makes S4 latch the brake. The run clears that itself before driving |
| **S2-C01** | Put a pen mark on each tyre. When asked, turn the **left** wheel exactly **10 turns** by hand (30 s), then the **right** wheel (30 s) | `S2-C01 LEFT: N edges in 10 turns -> B2_CAL_TICKS_PER_REV_LEFT = n` (and RIGHT) | Turn steadily in one direction. If the two numbers differ by more than about 2 %, run C01 again before believing them: both encoder discs should be identical |
| **S2-C02** | Put the robot down at a tape mark, pointing along the clear floor (15 s) | It drives **1000 mm** at 120 mm/s, then `edges L/R`, `L-R` and the encoder distance per wheel | Measure the distance **D** actually driven (front edge to front edge). **New circumference = current × D / 1000.** Also note the sideways drift at 1 m: it shows the straight-line correction |
| **S2-C03** | Stick tape along the robot's side to mark its heading (20 s) | It turns **left 360°** on the spot, then edges per wheel | Measure the angle **A** actually turned (a full turn plus or minus the gap from the tape). **New track = current × 360 / A** |
| **S2-C04** | Re-mark the heading (20 s) | It turns **right 360°** | Same formula. Use the **mean** of C03 and C04 |
| **S2-C05** | Clear 2 m ahead (20 s) | It steps from 0 to 150 mm/s for 2 s, one CSV line every 20 ms: `S2-STEP,t_ms,left_mm_s,right_mm_s,left_pwm,right_pwm,left_edges,right_edges` | Keep the log. `grep '^S2-STEP' log.txt` gives the step-response data for the PID tuning report |

Worked example: the car drove 1,032 mm in C02, so the circumference is
210 × 1032 / 1000 ≈ **217 mm**. It turned 341° in C03 and 345° in C04
(mean 343°), so the track is 120 × 360 / 343 ≈ **126 mm**.

## After the run

1. Put the new values in `motion_cal.h`. Rebuild the calibration image and run C02–C04 once more to check the error has dropped.
2. Record each step in [TEST_LOG.md](TEST_LOG.md): commanded value, measured value, error. Attach the console log under `logs/`.
3. Run the host tests (`sh INF2004_Project/testing/cr1/host_test/run_host_tests.sh`). Note that the "S5 constants unchanged" checks (4429 ticks/m, 46 ticks per 10°) are expected to fail after a re-calibration: update the expected numbers in `test_s2_motion.c` to the new values.

## PID tuning runs (several gain sets)

The PID tuning report compares step responses for several gain sets. To
try a set, pass it on the build command instead of editing the source
(values are Q8: the gain × 256, whole numbers). Anything left out keeps its
default from `app_program/subsystem2/tkernel_adapter.h` (kp 128, ki 8,
kd 0, straight 256).

```sh
make TEST=1 NET=lwip MQTT=1 S2_CAL=1 S2_KP=192 S2_KI=8 S2_KD=32 -j8
```

| Variable | Gain | Default |
|---|---|---|
| `S2_KP` | P, permil of PWM per mm/s of error × 256 | 128 |
| `S2_KI` | I, per control loop (~20 ms) × 256 | 8 |
| `S2_KD` | D, on the measured speed, per loop × 256 | 0 |
| `S2_SKP` | straight-line correction, mm/s per tick of L/R difference × 256 | 256 |

Before the `S2-STEP` lines, the run prints `S2-GAINS,kp_q8=…,ki_q8=…,kd_q8=…,straight_kp_q8=…`,
so every saved log records which gains it was made with. Changing the
gains rebuilds everything (the build profile changes), and the image name
stays the same, so flash straight after building. The demo build
(`make NET=lwip MQTT=1`) always uses the defaults unless you pass gains,
so once you have chosen a set, write it into `tkernel_adapter.h`.

The control loop runs every **~20 ms** (`tk_dly_tsk(10)` on the 10 ms
kernel tick), so the step log has one sample per loop at most; quote this
period in the report, because the I and D gains are per loop.

## Why the steps are in this order

The circumference only comes out right once each wheel's ticks per turn
are right (C01 before C02). The track only comes out right once the
circumference is right, because a turn is measured as a distance each
wheel rolls (C02 before C03/C04). If C01 changes the values, rebuild and
start again from C02.
