# Subsystem 2: PID tuning report

**Deliverable:** Buddy 2, "PID tuning report" (Project Write-up, p. 6).
**Code:** [`motion.c`](../../app_program/subsystem2/motion.c), gains in [`tkernel_adapter.h`](../../app_program/subsystem2/tkernel_adapter.h), limits in [`tkernel_adapter.c`](../../app_program/subsystem2/tkernel_adapter.c).
**Evidence:** host simulation ([`test_s2_motion.c`](../cr1/host_test/test_s2_motion.c), re-run 23 Nov 2026), floor telemetry logs ([`testing/subsystem1/logs/`](../subsystem1/logs/), analysed with [`analyse_logs.py`](../tools/analyse_logs.py)), and the 8 Nov bench session.

## 1. Summary

| Item | Value |
|---|---|
| Controller | Feed-forward + PI per wheel; D on measurement available but off. Separate straight-line correction between the wheels. |
| Loop period | ~20 ms (`tk_dly_tsk(10)` on a 10 ms kernel tick) |
| Chosen gains (Q8, value / 256) | **Kp = 128 (0.5)**, **Ki = 8 (0.031 per loop)**, **Kd = 0**, straight Kp = 256 (1.0) |
| Output range | 0 to 650 ‰ PWM duty (`max_pwm_permil`) |
| Speed range | 0 to 300 mm/s (`max_speed_mm_s`) |
| Step 0 → 150 mm/s (motor model) | peak 161 mm/s (7 % overshoot), settles at 147 mm/s (−2 %) |
| Floor tracking at 70 mm/s (13 Nov) | median 70 mm/s both wheels; p95 82 / 80 mm/s |
| Straight-line correction (motor model, 10 % weak motor) | drift 27 ticks (6.1 mm) → 1 tick (0.2 mm) over 5 s |

**Status:** the gains are tuned and justified on a motor model and confirmed on the floor at line-following speed. The on-floor step-response comparison of gain sets (S2-C05) is in §4.3.

## 2. Controller structure

Every loop, for each wheel `i`:

```
speed_i   = (3 × speed_i + raw_i) / 4                  // encoder edges this loop → mm/s, then a 1/4 EMA
target_i  = |request_i| ∓ trim                          // straight-line trim, ±20 mm/s max
error     = target_i − speed_i
integral  = clamp(integral + error, ±8000)              // anti-windup
duty      = FF + P + I + D
  FF      = 650 × target_i / 300                        // open-loop estimate of the duty needed
  P       = Kp × error / 256
  I       = Ki × integral / 256                         // ≤ ±250 ‰
  D       = −Kd × (speed_now − speed_prev) / 256        // on measurement: no kick on set-point steps
duty      = clamp(duty, 0, 650 ‰)
```

Design choices:

- **Feed-forward does most of the work.** The PI terms only correct the difference between the linear FF guess and the real motor (deadband, battery level, friction). This keeps the gains small and the response fast without much overshoot.
- **Integral clamp ±8,000** (×Ki 8/256 = ±250 ‰). Before 17 Nov it was ±20,000 (±625 ‰ of a 650 ‰ maximum). The integral could then drive the motor on its own, and the value stored while one wheel stalled pushed the next set-point far over target.
- **Integral reset** when the requested speed changes by more than 20 mm/s (`B2_INTEGRAL_RESET_MM_S`) or goes to 0. The integral belongs to the old set-point, and S3's line steering changes set-points constantly.
- **D on the measured speed, not the error,** so a set-point step from S3 gives no derivative kick.
- **Straight-line correction** (`b2_straight_trim`). When both wheels are asked for the same speed, the distance each has travelled since the segment started is compared. The wheel that is ahead is slowed and the one behind sped up, by Kp_straight × (ticks ahead), clamped to ±20 mm/s. It compares distance, not raw ticks, so it still works if the wheels end up with different ticks per turn.

## 3. Speed measurement limits the useful gains

With 930 encoder edges per revolution and a 210 mm circumference, one edge is **0.226 mm**. In a 20 ms loop that is **≈ 11 mm/s per edge**. At 70 mm/s a wheel sees only about 6 edges per loop, so the raw speed is coarse. The 1/4 EMA smooths it but adds lag.

Consequences for tuning:

- **A higher Kp amplifies quantisation noise** directly into the PWM. Kp = 0.5 ‰ per mm/s means one-edge noise (11 mm/s) moves the duty by about 6 ‰, which is small next to the FF term (≈ 150 ‰ at 70 mm/s).
- **D cannot help.** The derivative of an 11 mm/s-quantised signal is mostly noise. On the motor model, sweeping Kd from 0 to 8.0 changed the step peak by only 161 → 164 mm/s (host test comment, `test_s2_motion.c`). **Kd is therefore 0** in the demo build.

## 4. Tuning evidence

### 4.1 Host simulation (real `motion.c`, simulated motor)

The host test compiles the real `motion.c` against a motor model with a 60 ‰ deadband, a 100 ms first-order lag and a 10 % weaker right motor.

| Test | Gains | Result |
|---|---|---|
| Step 0 → 150 mm/s | Kp 0.5, Ki 0.031, **Kd 0** | peak **161 mm/s**, final **147 mm/s** |
| Step 0 → 150 mm/s | Kp 0.5, Ki 0.031, **Kd 2.0** | peak 163 mm/s, final 149 mm/s (D does not help) |
| Kd sweep 0 → 8.0 | | peak 161 → 164 mm/s: no meaningful change |
| Straight 5 s @ 100 mm/s, right motor 10 % weak | straight Kp **0** | L − R = **27 ticks (6.1 mm)** |
| same | straight Kp **1.0** | L − R = **1 tick (0.2 mm)**, end speeds 102 / 102 mm/s |
| Full suite | | 1,974 / 1,974 checks pass (23 Nov) |

### 4.2 On the robot

| Date | Conditions | Result | Source |
|---|---|---|---|
| 8 Nov | Bench, wheels lifted, 100 mm/s | Both wheels steady at target for 80 s | [bench report](../subsystem1/2026-11-08_bench_test_results.md), test 1 |
| 13 Nov | Floor, line following, target 70 mm/s, 451 straight-section samples | Left: median **70**, p95 82, max 96 mm/s. Right: median **70**, p95 80, max 98 mm/s | `2026-11-13_dashboard.csv` |
| 13 Nov | Floor, START → 90 % of cruise speed (n = 11) | median 748 ms, p95 2,791 ms | same log |

Notes on the floor figures:

- The median sits exactly on target for both wheels, so the integral removes steady-state error as intended.
- The p95 and max values include S3 steering corrections, which ask for different wheel speeds, so they are not pure controller error.
- START → 90 % includes S3's own start-up (line search and steering), so it is slower than the controller's own rise time. The 2.8 s tail is a start where the line had to be found first.
- The 1 Nov log was produced by a stub build with synthetic speeds and is not used.

### 4.3 Floor step response, gain sets compared (S2-C05)

Step 0 → 150 mm/s on the floor, one `S2-STEP` line per ~20 ms loop ([CALIBRATION.md](CALIBRATION.md), "PID tuning runs").

| Gain set | Kp | Ki | Kd | Rise to 90 % (ms) | Peak (mm/s) | Overshoot | Settled speed L / R (mm/s) | Stall fault? |
|---|---:|---:|---:|---:|---:|---:|---|---|
| Chosen (default) | 128 | 8 | 0 | | | | | |
| Higher P | 192 | 8 | 0 | | | | | |
| With D | 128 | 8 | 32 | | | | | |

## 5. Tuning history

| Date | Commit | Change | Reason |
|---|---|---|---|
| 3 Nov | `9ab0544` | FF + PI, Kp 128, Ki 8, limits 650 ‰ / 300 mm/s, stall 500 ms | First integrated version |
| 17 Nov | `18b1bd7` | Integral clamp ±20,000 → **±8,000**; integral reset on set-point change | Stored integral overshot the next set-point after a wheel stall |
| 19 Nov | `88c79df` | D term (on measurement) and straight-line correction added | Straight drift with unequal motors |
| 19 Nov | `aecb734` | Gains overridable at build time (`S2_KP`, `S2_KI`, `S2_KD`, `S2_SKP`); S2-STEP log in the calibration build | To compare gain sets on the floor without editing code |
| 20 Nov | `58493a2` | Stall-detection grace period after a wheel restarts | False ENCODER_STALL when a wheel reversed |

## 6. Limitations and remaining work

- How to run a gain set for §4.3: `make TEST=1 NET=lwip MQTT=1 S2_CAL=1 S2_KP=… S2_KI=… S2_KD=…`, flash, then `grep '^S2-STEP'` on the console log.
- The motor model is linear with one lag. It has no battery sag, load or floor friction, so the simulated overshoot is only indicative.
- The FF slope (650 ‰ for 300 mm/s) is a linear guess, not measured. A measured duty-to-speed curve would let Kp and Ki shrink further.
