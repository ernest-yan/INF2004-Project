# Subsystem 3 — test log

**Subsystem:** Line sensors + Code 39 barcode reader  ·  **Student ID:** 2500415  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | mapped characters give the right movement (F/U/L/R) | `get_action_for_character` | auto | `subsystem3_test()` in `line_barcode.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | unmapped and NUL characters give MOVE_NONE | `get_action_for_character` | auto | `subsystem3_test()` in `line_barcode.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | sensor snapshot readable while the task runs | mutex-protected data | auto | `subsystem3_test()` in `line_barcode.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | `mbf_barcode` exists | IF-01 | auto | `subsystem3_test()` in `line_barcode.c` | `[TEST] S3-U04 PASS` |
| S3-H01 | decoder on 3,000 simulated Code 39 scans (random letter, speed, narrow:wide ratio, ±25 % jitter, both directions, damaged scans) | decoder | auto (host) | `run_host_tests.sh` (`s3_driver.c`, real `line_barcode.c`) | 0 wrong letters; every undecoded scan reported as "seen, not decoded" |
| S3-H02 | barcode action at a cross junction (L, R, U-turn, straight, unmapped letter) on 8 simulated car variants + a course with a left stub | navigation command interface, junction detection | auto (host) | `run_host_tests.sh` (`s3_junction_sim.c`, real `line_barcode.c` with line following) | car leaves the junction on the arm the map asks for |
| S3-H03 | barcode decoded while line following (bar width ±1.4 mm, GP16 35–70 mm right of the line, wheel-speed noise, weaving start) | decoder + line following | auto (host) | as S3-H02 | 0 wrong letters |
| S3-M01 | a real Code 39 barcode is decoded | decoder | manual | flash the test build, pass a printed barcode in front of the right sensor (GP16) | console prints `[barcode] 'X'` with the right character |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-13 | S3-M01 | see log | floor run (log reviewed 23 Nov) | FAIL | [`2026-11-13_dashboard.csv`](../subsystem1/logs/2026-11-13_dashboard.csv) | Session 34179: barcode seen but reported INVALID (BARCODE event value 5) |
| 2026-11-13 | S3-M01 | see log | floor run (log reviewed 23 Nov) | PASS | [`2026-11-13_dashboard.csv`](../subsystem1/logs/2026-11-13_dashboard.csv), 18:14:34 | Session 46008: barcode decoded as LEFT (event value 1) while line following, matching the printed barcode (confirmed by the team); the car switched to BARCODE_NAV |
| 2026-11-26 | S3-H01 | `5901132` | host run | PASS | `run_host_tests.sh` output | 2,914 / 3,000 correct, 0 wrong; 86 undecoded, all reported (19 no frame, 67 bad letter) |
| 2026-11-26 | S3-H02 | `5901132` | host run | FAIL (40 / 41) | `run_host_tests.sh` output | All 40 runs on the plain cross correct. Left stub between barcode and junction: barcode R gave a U-turn at the stub. Known limitation, see [LINE_BARCODE.md §6.1](../../app_program/subsystem3/LINE_BARCODE.md) |
| 2026-11-26 | S3-H03 | `5901132` | host run | PASS | `run_host_tests.sh` output | 279 / 280 decoded, 0 wrong (1 not decoded at 20 % wheel-speed noise) |
