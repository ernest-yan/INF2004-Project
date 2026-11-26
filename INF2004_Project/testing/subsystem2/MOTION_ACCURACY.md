# Subsystem 2: motion accuracy evaluation

**Deliverable:** Buddy 2, "Motion accuracy evaluation" (Project Write-up, p. 6).
**Performance objectives evaluated:** consistent speed, accurate distance travelled, repeatable turns, stable motion.
**Code:** [`motion.c`](../../app_program/subsystem2/motion.c), calibration in [`motion_cal.h`](../../app_program/subsystem2/motion_cal.h).
**API:** `b2_move_forward(mm)`, `b2_move_backward(mm)`, `b2_turn_left(deg)`, `b2_turn_right(deg)`, `b2_stop()`. These are the Write-up's `moveForward()` … `stop()`.

## 1. Summary

| Objective | Evidence | Result |
|---|---|---|
| Consistent speed | Floor log, 13 Nov, 70 mm/s line following | Median on target (70 / 70 mm/s), p95 within +12 mm/s. **Met.** |
| Stable motion (straight) | Motor model, 10 % weak motor, 5 s | Drift 6.1 mm → 0.2 mm with correction (simulation). Floor drift at 1 m: see §3.4 |
| Accurate distance | Encoder resolution and host goal test | 0.226 mm per tick; goals end on the tick. Floor result: see §3.4 (S2-C02) |
| Repeatable turns | Host test, 90° turn | 417 / 417 ticks against a goal of 416. Floor result: see §3.4 (S2-C03/C04) |

The control side (speed, straight correction, goal completion) is shown to work. Floor distance and angle accuracy are in §3.4; the expected error budget is in §4.

## 2. Calibration values and where they come from

| Value | In use | Measured how | Confidence |
|---|---|---|---|
| Encoder edges per wheel revolution | 930 (both wheels) | 8 Nov, 10 hand turns per wheel: **L 962, R 896**. The 7 % gap is unconfirmed (the encoder discs should be identical), so the mean is used | Medium |
| Wheel circumference | 210 mm | Nominal tyre size | Corrected by S2-C02 (§3.4) |
| Wheel track (tyre centre to tyre centre) | 110 mm | 20 Nov, ruler | Good for geometry; the effective track on the floor may be a little wider because of tyre skid |

Derived constants (computed from `motion_cal.h`):

| Constant | Value |
|---|---|
| Distance per encoder edge | 0.226 mm |
| Encoder edges per metre | 4,429 |
| Edges per wheel per 10° of on-the-spot turn | 43, i.e. about 0.23° per edge |

**Floor cross-check of the encoder gap.** On the 13 Nov floor run the straight sections gave L 28,275 and R 27,124 edges, a ratio R/L = 0.959. If both wheels roll the same distance, that ratio equals R/L edges per turn. It points the same way as the hand-turn result (896 / 962 = 0.931) but is smaller. Line following is not perfectly straight, so this supports a real per-wheel difference without pinning its size. S2-C01 should be repeated before per-wheel values are used.

## 3. Results

### 3.1 Speed consistency (floor, 13 Nov, target 70 mm/s, 451 straight-section samples)

| Wheel | Median | p95 | Max |
|---|---:|---:|---:|
| Left | 70 mm/s | 82 mm/s | 96 mm/s |
| Right | 70 mm/s | 80 mm/s | 98 mm/s |

The p95 and max include S3's steering corrections, so the spread is an upper bound on controller error. On the bench (8 Nov, wheels lifted) both wheels held 100 mm/s for 80 s.

### 3.2 Straight-line stability (host simulation)

The real `motion.c` drives a motor model with a 60 ‰ deadband, a 100 ms lag and the **right motor 10 % weaker**, at 100 mm/s for 5 s.

| Straight-line correction | L − R travel | End speeds |
|---|---:|---|
| Off | 27 ticks (6.1 mm) | |
| On (Kp 1.0, ±20 mm/s) | **1 tick (0.2 mm)** | 102 / 102 mm/s |

### 3.3 Goal completion (host simulation)

| Command | Goal | Result |
|---|---|---|
| `b2_turn_left(90)` (test config, 120 mm track) | 416 ticks per wheel | L 417, R 417: both wheels finish together, within 1 tick (≈ 0.2°) |
| `b2_move_forward(…)` | distance at the requested speed | Completes at the goal distance (S2 host test, MOVE goal checks) |
| Out-of-range requests (0 mm, > 10.23 s of travel, > 360°) | | Refused, never truncated (S2-U07) |

### 3.4 Floor distance and turn accuracy

| Test | Commanded | Measured | Error | Target |
|---|---|---|---|---|
| S2-C02 straight | 1,000 mm | | | within 2 % after calibration |
| S2-C02 sideways drift at 1 m | 0 mm | | | recorded |
| S2-C03 turn left | 360° | | | within 5° after calibration |
| S2-C04 turn right | 360° | | | within 5° after calibration |

The calibration build and procedure are ready in [CALIBRATION.md](CALIBRATION.md).

## 4. Expected error budget

Encoder-based motion is only as accurate as the calibration:

- **Distance error % = circumference error %.** Every 1 % error in the 210 mm circumference is a 10 mm error over 1 m. The circumference comes from S2-C02, so it is the value that matters most.
- **Turn error % ≈ circumference error % + track error %.** A turn is a distance each wheel rolls (circumference), around a circle set by the track. If skid makes the effective track 5 mm wider than the 110 mm measured, a 90° turn comes out about 4° short.
- **Per-wheel edge difference.** If the 962 / 896 hand-turn gap is real, using the 930 mean makes each wheel's distance off by about ±3.5 %. While both wheels use 930, the straight-line correction equalises their tick counts, so a real gap would make the car curve slightly instead of drifting at random. Setting the per-wheel values in `motion_cal.h` removes that, because the correction compares distance per wheel. On a turn the gap gives an error of a few degrees.
- **Resolution** (0.226 mm, 0.23° per edge) is not a limiting factor.

## 5. Remaining work

1. Repeat S2-C01 (hand turns) to confirm or reject the per-wheel difference.
2. Run S2-C02 and correct the circumference: new = 210 × measured / 1000.
3. Run S2-C03 and S2-C04 and correct the track: new = 110 × 360 / mean angle.
4. Re-run C02 to C04 with the new values and fill in the table in §3.4. Record the runs in [TEST_LOG.md](TEST_LOG.md).
