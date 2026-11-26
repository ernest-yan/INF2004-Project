# Subsystem 4 — test log

**Subsystem:** Accelerometer (GY-511)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | LSM303DLHC answers on I2C1 GP26/GP27 | I2C1 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | calibration completes, acceleration magnitude 0.7-1.3 g at rest | sensor | auto | `subsystem4_test.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | synthetic 10 deg hump detected, 13-15 mm | terrain maths | auto | `subsystem4_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | level travel never reports a hump | terrain maths | auto | `subsystem4_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | humps, vibration, edge shock: no false lift | posture maths | auto | `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | upward 0.4 g held -> LIFTED within 100 ms | posture maths | auto | `subsystem4_test.c` | `[TEST] S4-U06 PASS` |
| S4-U07 | held at 40 deg tilt -> LIFTED | posture maths | auto | `subsystem4_test.c` | `[TEST] S4-U07 PASS` |
| S4-U08 | upside down -> OVERTURNED | posture maths | auto | `subsystem4_test.c` | `[TEST] S4-U08 PASS` |
| S4-U11 | gentle 20 cm / 1.5 s lift in vibration -> LIFTED within 0.75 s | posture maths | auto | `subsystem4_test.c` | `[TEST] S4-U11 PASS` |
| S4-U12 | put down with a landing thump: no lift | posture maths | auto | `subsystem4_test.c` | `[TEST] S4-U12 PASS` |
| S4-M01 | lifting the robot stops the motors (end of run) | IF-04 | manual | normal image, robot driving on the line, pick it up | motors stop at once; `[VC] S2 st=3` (BRAKED), PWM off; stays stopped when put down until RESET |
| S4-M02 | overturning the robot stops the motors | IF-04 | manual | normal image, wheels driven, turn the robot on its side | motors stop; S4 lift cause = overturned |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-16 | S4-M01 | see log | floor run (log reviewed 23 Nov) | PARTIAL (4 of 5) | [`2026-11-16_dashboard_groupmate.csv`](../subsystem1/logs/2026-11-16_dashboard_groupmate.csv) | 5 E-STOPs with source IMU (lift) at 16:02:51, 16:05:28, 16:09:57, 16:35:04, 16:53:59 (each event logged twice). In 4 both wheels read 0 mm/s within 2 s. At 16:09:57 the left wheel went 601 → 62 mm/s and still read 62 mm/s 2 s later. Whether every trigger was a real pick-up is not recorded; 16:35:04 came right after an obstacle scan. Early (pre-commit) version of lift detection |
