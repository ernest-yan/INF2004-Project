# Subsystem 1 — test log

**Subsystem:** WiFi telemetry and remote commands (MQTT)  ·  **Student ID:** 2502568  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | outgoing message buffer exists | IF-14 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | START command packs to `11 4A 3B 00 01` | MQTT protocol | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | valid command accepted | MQTT protocol | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | replayed command rejected (STALE_SEQ) | MQTT protocol | auto | `subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | old-session / retained command rejected | MQTT protocol | auto | `subsystem1_test.c` | `[TEST] S1-U05 PASS` |
| S1-U06 | offline (session 0) rejects everything | MQTT protocol | auto | `subsystem1_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | wrong length rejected | MQTT protocol | auto | `subsystem1_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | unknown command code rejected | MQTT protocol | auto | `subsystem1_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | STATUS frame is 17 bytes, big values clamp | Minimum-Bit | auto | `subsystem1_test.c` | `[TEST] S1-U09 PASS` |
| S1-U10 | encoder refuses a too-small buffer | Minimum-Bit | auto | `subsystem1_test.c` | `[TEST] S1-U10 PASS` |
| S1-U11 | command queue exists and holds exactly 4 commands | IF-13 | auto | `subsystem1_test.c` | `[TEST] S1-U11 PASS` |
| S1-U12 | telemetry task running at priority 13 | TEAM.md §4 | auto | `subsystem1_test.c` | `[TEST] S1-U12 PASS` |
| S1-U13 | telemetry task only ever runs on core 0 | TEAM.md §4 (S3 owns core 1) | auto | `subsystem1_test.c` (SMP build only) | `[TEST] S1-U13 PASS` |
| S1-U14 | S2 ready within 3 s (precondition for U15–U17) | IF-09 | auto | `subsystem1_test.c` | `[TEST] S1-U14 PASS` |
| S1-U15 | vehicle state worked out from S2–S5, faults only in bits 2–5 | IF-09…IF-12 | auto | `subsystem1_test.c` | `[TEST] S1-U15 PASS` |
| S1-U16 | real S2 speeds read 0 with the robot at rest | IF-09 | auto | `subsystem1_test.c` | `[TEST] S1-U16 PASS` |
| S1-U17 | no S2 deadline misses in 3 s with telemetry running | IF-09, TEAM.md §4 | auto | `make TEST=1 NET=lwip MQTT=1`, **broker running** so telemetry is ONLINE (check the `[TEST] S1 note` line) | `[TEST] S1-U17 PASS`, note shows `+0` |
| S1-U18 | telemetry loop always under 2 ms | TEAM.md §4 | auto | as S1-U17 | `[TEST] S1-U18 PASS` |
| S1-M01 | dashboard shows real wheel data | IF-09 | manual | wheels lifted, normal MQTT image, `dashboard.py` running; turn each wheel by hand | `enc_l` / `enc_r` rise for their own wheel; speeds non-zero while turning |
| S1-M02 | dashboard shows real line sensors | IF-10 | manual | hold black tape under left, then centre, then both sensors | `line` = 2, 1, 3 (0 off the line) |
| S1-M03 | dashboard shows real pitch | IF-11 | manual | tilt the robot nose-up ~20° | `pitch` follows the tilt (within a few degrees) |
| S1-M04 | dashboard shows real obstacle distance | IF-12 | manual | hand in front of the HC-SR04 at ~30 cm, then remove it | `obs_cm` ≈ 30, then 511 (no echo) or the far wall |
| S1-M05 | remote command handled by our task, not the WiFi task | IF-13 | manual | send START, STOP, E-STOP from the dashboard | ACK `ACCEPTED`; console shows `[S1] remote START …` etc.; MQTT stays connected |
| S1-M06 | no motor slow-down over a long run | IF-09, TEAM.md §4 | manual | MQTT image online for 5 min, wheels lifted; read Buddy 2's `deadline_misses` (S2 status) before and after | count unchanged |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

Note: remote START / STOP / E-STOP only log a message until the team
decides what they do (S1-M05 checks the path, not the robot's reaction).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
