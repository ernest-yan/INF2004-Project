# Subsystem 4: terrain analysis report

**Deliverable:** Buddy 4, "Terrain analysis report" (Project Write-up, p. 8).
**Code:** [`imu.c`](../../app_program/subsystem4/imu.c) (driver, calibration, 100 Hz loop), [`terrain.c`](../../app_program/subsystem4/terrain.c) (humps), [`posture.c`](../../app_program/subsystem4/posture.c) (lift / overturn), [`motion_class.c`](../../app_program/subsystem4/motion_class.c) (motion events), config in [`tkernel_adapter.c`](../../app_program/subsystem4/tkernel_adapter.c) / [`.h`](../../app_program/subsystem4/tkernel_adapter.h).
**Sensor:** GY-511 (LSM303DLHC) accelerometer on I2C1 (GP26 / GP27), 100 Hz, ±2 g. The module has no gyroscope, so all angles come from the gravity vector, and turn rate comes from the wheel encoders.

## 1. Summary

| Task (Write-up) | Implemented | Evidence | Status |
|---|---|---|---|
| IMU integration | LSM303DLHC driver, 100 Hz, fault bit on I2C error | Floor logs since 4 Nov | ✅ |
| Sensor calibration | 50 still samples at boot, restarts if the car moves | Host simulation | ✅ |
| Tilt detection | Pitch and total tilt from gravity, low-pass filtered | Floor log 13 Nov: driving \|pitch\| median 0°, p95 11° | ✅ |
| Hump detection | Peak-tilt method, gated on wheel travel | 12 / 12 simulated humps; floor calibration 17 Nov | ✅ |
| Peak hump measurement | `max_hump_height_mm`, sent in every status frame | 17 Nov calibration: 6 crossings of the 10 mm hump | ⚠️ calibrated on one hump height only |
| Motion classification | 7 classes (§5) | Host simulation | ✅ |
| Collision detection | Implemented as **lift / overturn → latched brake** (end-of-run rule) | 16 Nov floor: 8 IMU brakes; host: 24 / 24 simulated stops | ✅ (thresholds provisional) |
| Turn-rate monitoring | From wheel-speed difference | Code | ✅ |
| Motion event reporting | HUMP, LIFTED, OVERTURNED events over MQTT | Floor logs | ✅ |

## 2. Calibration

At power-on the car must stand still. S4 averages **50 samples (0.5 s)** of the acceleration vector:

- The mean vector becomes "up" for the lift and overturn rules.
- The mean pitch becomes the **pitch offset**, so a slightly tilted mounting reads 0° on level floor.
- If the acceleration magnitude varies by more than **100 mg** during the window, the car was moved and calibration restarts. Restarts are counted in the status.

## 3. Hump detection and height

### 3.1 Method (peak-tilt)

A hump shorter than about two wheelbases does not lift the whole car; the car **rocks** over it. When the front wheels are on top and the rear is still on the floor, the car tilts nose-up by

```
sin(tilt) = hump height / wheelbase
```

and when the rear wheels reach the top it tilts nose-down by the same amount. So

```
height = wheelbase × (sin(peak nose-up) + sin(peak nose-down)) / 2
```

Implementation details ([`terrain.h`](../../app_program/subsystem4/terrain.h)):

| Rule | Value | Why |
|---|---|---|
| Start | tilt > 2.0° for 80 ms **while the wheels turn** | Picking up or moving a stopped car never counts as a hump |
| Solid half | tilted over ≥ 30 mm of travel | Pulling away or braking tilts the car over only ~10 mm |
| End | 80 mm of travel with no clear tilt, or stopped for 1 s | |
| Reject | longer than 600 mm (a ramp), height < 3 mm or > 150 mm | |
| Halves | Each half used alone when solid; averaged when both are | On this car the small front caster barely tilts it, while the big rear drive wheels give a clear nose-down half |
| Cross-check | the travel between the two peaks should equal the wheelbase (`peak_gap_mm`) | |

### 3.2 Why the method changed (old logs)

The first version (3 Nov) added up distance × slope, which measures the **average** height under the car, about 6 mm for the course's 10 mm hump. It also counted tilt while the car was being handled. The 13 Nov floor log shows what that produced: `hump_max_mm` took the values **21, 38, 94 and 124 mm** during the session, against a ~10 mm hump. Those readings came from handling the car and from long tilted sections, not from the hump.

The peak-tilt method (17 Nov) fixes both: only tilt **while the wheels turn** counts, and the height comes from the peak tilt, not an integral.

### 3.3 Floor calibration (17 Nov)

The height scales directly with the wheelbase constant. It was calibrated on the course hump:

| Run | Hump | Crossings | Readings | Mean |
|---|---|---|---|---|
| Wheelbase constant = 96 mm (previous value) | 10 mm course hump | 6 | 6 to 10 mm | 7.3 mm |
| Corrected constant | | | | 96 × 10 / 7.3 = **131 mm** |

The constant is an **effective** wheelbase: it also absorbs the low-pass filter on pitch, which delays and lowers the tilt peak. It must be re-checked if the IMU mounting or the driving speed changes. The spread of the six readings (6 to 10 mm, about ±2 mm) gives an estimate of the measurement repeatability.

### 3.4 Host simulation

The same `imu.c` / `terrain.c` code, run on 12 simulated runs with humps from 2.5° to 10° plus turns, lifts and an I2C fault ([`s4_driver.c`](../cr1/host_test/s4_driver.c), re-run 23 Nov):

- **12 / 12 humps** detected and measured.
- No extra humps: exactly one hump reported per run, so level travel and turns gave none.
- Output identical, line for line, before and after the CR1 bit-packing change (2,074 status and event lines).

### 3.5 Accuracy on other hump heights

Crossings at line-following speed with the calibrated constant (131 mm), `hump_max_mm` / HUMP event from the dashboard.

| Hump height (measured with a ruler) | Crossings | Readings | Mean | Error |
|---|---:|---|---:|---:|
| 10 mm (course hump) | | | | |
| | | | | |
| | | | | |

## 4. Lift / overturn detection (collision rule)

The Week 13 rubric ends the run when the car is picked up ("stop motors when the car is picked up"). S4 detects this and latches the S2 emergency brake (interface IF-04). The brake stays on until a remote RESET.

| Cause | Rule | Threshold (provisional) |
|---|---|---|
| Lifted: moving up | vertical speed (acceleration integrated with a 1 s leak) above the limit for 100 ms | 150 mm/s |
| Lifted: sharp jolt | upward acceleration above 1 g + limit for 50 ms | 250 mg |
| Lifted: carried tilted | tilt above the limit for 200 ms | 30° |
| Overturned | gravity more than the limit away from calibrated "up" for 100 ms | 60° |

False-trigger protection: low-pass filters and hold times reject single shocks. For 1 s after a clear downward movement (putting the car down), the two "upward" rules are ignored, because a landing looks like an upward push. The 30° tilt limit must stay above the steepest hump on the course.

Evidence:

| Source | Result |
|---|---|
| Host simulation | 24 / 24 simulated lift and overturn cases stop the motors, and the brake stays latched |
| On-target unit tests | S4-U05 to S4-U12 (humps, vibration, edge shock: no false lift; gentle and sharp lifts, tilt, overturn) are defined in `subsystem4_test.c`; no results logged |
| 16 Nov floor log (early version) | 8 emergency brakes with source = IMU (lift) |
| 4 Nov floor log (old impact rule, since replaced) | 4 IMPACT events |

**Known limits.** A very slow lift (20 cm over more than ~2 s) never reaches the speed threshold; an accelerometer alone cannot see it, but the 30° tilt rule catches it if the car is tilted. On 16 Nov one IMU brake came straight after an obstacle scan, so a false trigger during S5's manoeuvres has not been ruled out.

## 5. Motion classification and turn rate

Classes, most important first:

| Class | Condition |
|---|---|
| CALIBRATING | calibration not finished |
| OVERTURNED / LIFTED | posture rule above |
| ON_HUMP | hump phase is climbing or descending |
| STATIONARY | both wheels slower than 20 mm/s |
| TURNING | \|yaw rate\| ≥ 45°/s |
| MOVING | otherwise |

Yaw rate = (right speed − left speed) / track, from the encoders. The track comes from S2's calibration (`B2_CAL_WHEEL_TRACK_MM`, 110 mm). The 45°/s limit is provisional: above line-following corrections, below on-the-spot turns.

The Write-up's "accelerating" and "descending hump" events are covered by the hump phase (CLIMB / DESCEND, sent on the wire) and S2's speed; "sudden impact" was replaced by lift / overturn on 18 Nov because the Week 13 rubric uses pick-up as the end-of-run signal.

## 6. Floor tilt observed (13 Nov, driving)

| Measure | Value |
|---|---|
| \|pitch\| median | 0° |
| \|pitch\| p95 | 11° |
| \|pitch\| max | 33° |

The 33° maximum is above today's 30° lift-tilt limit. That session ran an older build without today's lift rules, so whether it would have tripped the 200 ms hold is not known.

## 7. Limitations and remaining work

1. **Hump height is calibrated on one hump (10 mm)**; results on other heights are in §3.5. The method is accurate only while the hump's half-length is no longer than the wheelbase.
2. The 17 Nov calibration readings are documented in the `S4_WHEELBASE_MM` comment; floor logs of the new method go with §3.5.
3. Lift and overturn thresholds are provisional (S4-M01, S4-M02 in the S4 TEST_LOG).
4. **Fixed 23 Nov:** `S4_WHEEL_TRACK_MM` was a separate 120 mm while S2's track was re-measured at 110 mm on 20 Nov, so S4's yaw rate read about 9 % low and TURNING triggered late. S4 now takes the track from S2's `motion_cal.h`, so the two cannot drift apart. The change compiles with no warnings.
