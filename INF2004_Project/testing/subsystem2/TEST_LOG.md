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
| S2-M01 | encoders count | GP6/GP7 | manual | wheels lifted, turn each wheel by hand, watch `[VC] S2 ... enc=L/R` | both counts rise, each only for its own wheel |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
