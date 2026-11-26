# Subsystem 2 — test log

**Subsystem:** Motion control  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | motion task configured PWM/encoder IRQ (b2_init OK on core 0) | IF-03 | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | idle after boot: READY, no fault, PWM zero | IF-03 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | mm <-> encoder-tick maths | calibration | auto | `subsystem2_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | invalid speed / zero timeout / NULL command rejected | IF-03 | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | emergency brake latches and blocks commands | IF-04 | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | authorised reset while still returns to READY | IF-04 | auto | `subsystem2_test.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | move/turn API refuses 0, too far (> 10.23 s), > 360° | Write-up API (`moveForward` …) | auto | `subsystem2_test.c` (no wheel moves) | `[TEST] S2-U07 PASS` |
| S2-U08 | S5's tick scales come from S2's calibration header | `motion_cal.h` | auto | `subsystem2_test.c` | `[TEST] S2-U08 PASS` |
| S2-M01 | encoders count | GP6/GP7 | manual | wheels lifted, turn each wheel by hand, watch `[VC] S2 ... enc=L/R` | both counts rise, each only for its own wheel |
| S2-C01 | encoder edges per wheel turn, each wheel | calibration | manual | `make TEST=1 NET=lwip MQTT=1 S2_CAL=1`, [CALIBRATION.md](CALIBRATION.md) | L and R within ~2 % of each other; values go in `motion_cal.h` |
| S2-C02 | 1000 mm straight: distance and drift | calibration, straight-line correction | manual | as S2-C01 | measured distance within 2 % after calibration; sideways drift recorded |
| S2-C03 | 360° left turn on the spot | calibration, encoder turning | manual | as S2-C01 | measured angle within 5° after calibration |
| S2-C04 | 360° right turn on the spot | calibration, encoder turning | manual | as S2-C01 | as S2-C03 |
| S2-C05 | speed step 0 → 150 mm/s, 50 Hz log | PID tuning | manual | as S2-C01; keep the `S2-STEP` console lines | settles near 150 mm/s on both wheels, no stall fault |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-08 | S2-C01 | see `motion_cal.h` | hand-turn session | FAIL | comment in [`motion_cal.h`](../../app_program/subsystem2/motion_cal.h) (no console log saved) | 10 turns per wheel: L 962, R 896 edges per turn, a 7 % gap against the ~2 % expected. Unconfirmed, so both wheels use the mean 930. Re-run before using per-wheel values |
| 2026-11-13 | S2-M01 | see log | floor run (log reviewed 23 Nov) | PASS | [`2026-11-13_dashboard.csv`](../subsystem1/logs/2026-11-13_dashboard.csv) | Both encoders count while driving: L 28,275 / R 27,124 edges on straight sections, each wheel's count rising only with its own speed. From floor data, not the wheels-lifted hand-turn procedure |
