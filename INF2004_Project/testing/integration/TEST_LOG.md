# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | decoded characters from S3 reach a receiving task | IF-01 | manual | scan a barcode in front of GP16 | `[barcode] 'X'` with the same character |
| INT-02 | obstacle stops the robot and S5 completes its scan | IF-02, IF-03, IF-06 | manual | robot still, hold a box 25 cm in front of the HC-SR04 | `[VC] S5 obstacle ... -> stop`, servo sweeps, then `[VC] S5 profile: ...` (no STOP_TIMEOUT fault) |
| INT-03 | impact latches the S2 brake | IF-04 | manual | see S4-M01 | S2 state BRAKED |
| INT-04 | odometry reaches S4 | IF-05 | manual | wheels lifted, turn both wheels by hand for ~1 m of travel | S4 terrain stays LEVEL on flat; no hump reported |
| INT-05 | microsecond timer runs at 1 MHz | S3/S5 timing | auto | boot the normal image | `[VC] microsecond timer OK` |
| INT-06 | robot follows the line | IF-03 | manual | wheels lifted first, then on the track: normal image, keep still until `[S3] line following armed`, then place on the line | follows the line; `[S3] line lost, stopped` when lifted off it |
| INT-07 | stored barcode runs at the next cross-junction | IF-01, IF-03 | manual | follow the line over a barcode in each scan direction, then reach a cross-junction; repeat for each of the four movements | `[S3] barcode 'L' stored`, no movement until the junction, then `[S3] junction, barcode 'L' -> action ...` and the matching movement |
| INT-08 | robot bypasses an obstacle and returns to the line | IF-02, IF-03, IF-07, IF-08 | manual | box on the line, clear space on one side | stops, servo sweeps, `[S5] bypassing on the ...`, `[S5] bypass done, back on the line`, then `[S3] line found, following` |
| INT-09 | S2 wheel data reaches the dashboard | IF-09 | manual | see S1-M01 | dashboard encoders/speeds match `[VC] S2 ... enc=L/R` |
| INT-10 | S3 line sensors reach the dashboard | IF-10 | manual | see S1-M02 | dashboard `line` matches `[VC] ... S3 L=… C=…` |
| INT-11 | S4 pitch reaches the dashboard | IF-11 | manual | see S1-M03 | dashboard `pitch` ≈ `[VC] ... S4 pitch=…cdeg` / 100 |
| INT-12 | S5 distance reaches the dashboard | IF-12 | manual | see S1-M04 | dashboard `obs_cm` ≈ `[VC] ... S5 front=…` / 10 |
| INT-13 | telemetry does not slow S2's motor loop | IF-09, priorities | auto + manual | S1-U17 (auto), S1-M06 (5 min) | S2 `deadline_misses` unchanged |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
