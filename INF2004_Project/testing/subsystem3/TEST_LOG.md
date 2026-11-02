# Subsystem 3 — test log

**Subsystem:** Line sensors + Code 39 barcode reader  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | mapped characters give the right movement (F/U/L/R) | `get_action_for_character` | auto | `subsystem3_test()` in `line_barcode.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | unmapped and NUL characters give MOVE_NONE | `get_action_for_character` | auto | `subsystem3_test()` in `line_barcode.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | sensor snapshot readable while the task runs | mutex-protected data | auto | `subsystem3_test()` in `line_barcode.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | `mbf_barcode` exists | IF-01 | auto | `subsystem3_test()` in `line_barcode.c` | `[TEST] S3-U04 PASS` |
| S3-M01 | a real Code 39 barcode is decoded | decoder | manual | flash the test build, pass a printed barcode in front of the right sensor (GP16) | console prints `[barcode] 'X'` with the right character |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
