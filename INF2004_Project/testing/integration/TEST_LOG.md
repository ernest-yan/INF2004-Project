# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | decoded characters from S3 reach a receiving task | IF-01 | manual | scan a barcode in front of GP16 | `[VC] barcode 'X'` with the same character |
| INT-02 | obstacle stops the robot and S5 completes its scan | IF-02, IF-03, IF-06 | manual | robot still, hold a box 25 cm in front of the HC-SR04 | `[VC] S5 obstacle ... -> stop`, servo sweeps, then `[VC] S5 profile: ...` (no STOP_TIMEOUT fault) |
| INT-03 | impact latches the S2 brake | IF-04 | manual | see S4-M01 | S2 state BRAKED |
| INT-04 | odometry reaches S4 | IF-05 | manual | wheels lifted, turn both wheels by hand for ~1 m of travel | S4 terrain stays LEVEL on flat; no hump reported |
| INT-05 | microsecond timer runs at 1 MHz | S3/S5 timing | auto | boot the normal image | `[VC] microsecond timer OK` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
