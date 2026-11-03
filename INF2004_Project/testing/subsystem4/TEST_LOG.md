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
| S4-M01 | impact brakes the motors | IF-04 | manual | normal image, robot still, tap the chassis hard | `[VC] S2 st=3` (BRAKED), PWM off |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
