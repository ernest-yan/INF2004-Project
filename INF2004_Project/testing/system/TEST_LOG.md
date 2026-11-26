# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

SYS-01 to SYS-10 are the ten points of the **Week 13 demo rubric** (posted 16 Nov 2026), checked on a full course run. Requirement IDs (FR, NFR) are from the design review.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Detect a barcode | FR2 | manual | Full course run with the dashboard logging; robot follows the line over a barcode | BARCODE event (or "seen, not decoded" scan) on the dashboard as the robot passes it |
| SYS-02 | Decode the barcode and obtain the code | FR2 | manual | As SYS-01; note the printed barcode's letter beforehand | BARCODE event carries the right letter |
| SYS-03 | Measure hump height | FR4, FR5 | manual | Course run over a hump of known height | `hump_max_mm` on the dashboard close to the known height |
| SYS-04 | Follow the line | FR1 | manual | Course run from the start | Robot stays on the line to the end of the track section |
| SYS-05 | Follow the instruction from the barcode | FR3 | manual | Send `map` before the run; course run over a barcode to a junction | Robot performs the mapped movement (left, right, straight or U-turn) and continues on the line |
| SYS-06 | Detect an obstacle | FR6 | manual | Obstacle on the line ahead | Robot stops or slows before reaching it; OBSTACLE_AVOID on the dashboard |
| SYS-07 | Measure obstacle width | FR7 | manual | As SYS-06, with an obstacle of known width | OBSTACLE event width close to the real width |
| SYS-08 | Navigate around the obstacle and rejoin the line | FR8, FR9 | manual | As SYS-06, clear space on one side | Robot bypasses the obstacle without contact and returns to LINE_FOLLOW |
| SYS-09 | Send telemetry data via MQTT | FR10, NFR3 | manual | Any run with `dashboard.py` logging | Status frames at ~5 Hz and heartbeat at ~1 Hz throughout the run |
| SYS-10 | Stop motors when the car is picked up (end of demo) | FR12 | manual | Pick the robot up while it drives | Motors stop at once and stay stopped (E-STOP source IMU) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

The rows below are from the **pre-demo floor runs** (13 and 16 Nov) and come from the dashboard logs. They are not the Week 13 demo score; add the demo run's result as new rows. Rubric items with no evidence in the logs (SYS-05, SYS-07) have no row yet.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-13 | SYS-01 | see log | floor run (log reviewed 23 Nov) | PASS | [`2026-11-13_dashboard.csv`](../subsystem1/logs/2026-11-13_dashboard.csv) | Two BARCODE events in the session (one INVALID, one LEFT) |
| 2026-11-13 | SYS-02 | see log | floor run (log reviewed 23 Nov) | PASS | as above, 18:14:34 | Decoded as LEFT, matching the printed barcode (confirmed by the team). An earlier scan in the session was INVALID. See S3-M01 |
| 2026-11-13 | SYS-03 | see log | floor run (log reviewed 23 Nov) | FAIL | as above | Old hump algorithm: `hump_max_mm` reached 21, 38, 94 and 124 mm against a ~10 mm hump. Replaced on 17 Nov by the peak-tilt method (6 crossings read 6–10 mm after calibration, see [TERRAIN_ANALYSIS.md](../subsystem4/TERRAIN_ANALYSIS.md)); not re-logged on the floor |
| 2026-11-13 | SYS-04 | see log | floor run (log reviewed 23 Nov) | PASS (jittery) | as above | See INT-06: longest continuous LINE_FOLLOW 51.8 s, but frequent short drops to LINE_SEARCH (median stretch 0.60 s) |
| 2026-11-13 | SYS-06 | see log | floor run (log reviewed 23 Nov) | PASS | as above | 9 obstacle stops triggered at 20–36 cm; OBSTACLE_AVOID entered 13 times |
| 2026-11-16 | SYS-08 | see log | floor run (log reviewed 23 Nov) | FAIL | [`2026-11-16_dashboard_groupmate.csv`](../subsystem1/logs/2026-11-16_dashboard_groupmate.csv); also 13 Nov log | Every OBSTACLE_AVOID ended in SAFE_STOP (13 Nov: 12 of 13, 16 Nov: 6 of 6). S5 bypass rewritten on 20 Nov (`58493a2`), not re-logged |
| 2026-11-13 | SYS-09 | see log | floor run (log reviewed 23 Nov) | PASS | 13 Nov log; also 16 Nov log | Status interval median 195 ms (13 Nov) / 202 ms (16 Nov), heartbeat median 1,017 / 964 ms. 9.6 % / 1.5 % of status frames lost, around Wi-Fi reconnects. See S1 TEST_LOG |
| 2026-11-16 | SYS-10 | see log | floor run (log reviewed 23 Nov) | PARTIAL (4 of 5) | 16 Nov log | See S4-M01: wheels at 0 mm/s within 2 s after 4 of 5 IMU lift E-STOPs |
