# Subsystem 3: line following, barcode decoder and navigation command interface

**Deliverables:** Buddy 3, "Line-following module", "Barcode decoder" and "Navigation command interface" (Project Write-up, p. 7).
**Code:** [`line_barcode.c`](line_barcode.c) / [`line_barcode.h`](line_barcode.h); on-target tests in [`line_barcode_test.c`](line_barcode_test.c).
**Sensors:** three digital IR reflectance sensors (LM393 comparator boards, active HIGH on black): **GP2** left line sensor, **GP4** centre line sensor, **GP16** right barcode sensor.
**Tests and results:** [`testing/subsystem3/TEST_LOG.md`](../../testing/subsystem3/TEST_LOG.md).

## 1. Summary

| Task (Write-up) | Implemented | Evidence | Status |
|---|---|---|---|
| IR sensor integration | GP2, GP4, GP16 polled by `barcode_task` (core 1, no sleep) | Floor logs since 4 Nov | ✅ |
| Sensor calibration | Hardware threshold: each LM393 board's trimmer pot sets the black/white level, so the firmware reads a clean 0/1 | Set by hand on the course surface | ⚠️ no software calibration (§6) |
| Line position estimation | Four positions from two sensors: centred, line to the left, line to the right (seen by GP16), lost (§2.2) | Host simulator | ✅ coarse (§6) |
| Line-following algorithm | Bang-bang steering with a soft arc, a hard turn after 400 ms, a soft exit and a two-way search (§2) | Floor log 13 Nov, host simulator | ✅ jittery on the floor |
| Junction detection | GP2 **and** GP4 on black for 40 ms | Host simulator: 40 / 41 runs | ✅ one known failure (§6) |
| Barcode detection | GP16 edge timing; a scan ends after 400 ms with no edge | Floor 13 Nov: 2 scans detected | ✅ |
| Barcode decoding | Code 39, framed by `*` … `*`, both directions, bar-widening correction (§3) | Host: 2,914 / 3,000 correct, **0 wrong**; floor: LEFT decoded on 13 Nov | ✅ |
| Navigation command generation | Letter → movement through the MQTT `map`; executed at the next junction (§4) | Host simulator: L / R / U / F correct at the junction | ✅ |

| Performance objective | Result |
|---|---|
| Reliable line tracking | Floor (13 Nov): followed the line for up to 51.8 s without a break, but most stretches were short (median 0.60 s) before a brief search. It always recovered. |
| Accurate barcode recognition | Host: **0 wrong letters** in 3,280 simulated scans. A barcode that cannot be read gives "seen, not decoded", never a wrong turn. |
| Robust operation at varying speeds | Host: random speed per scan (s3_driver), and ±20 % wheel-speed noise while line following still decodes 19 / 20 with 0 wrong. |

## 2. Line-following module

### 2.1 Tasks

| Task | Core | Priority | Period | Job |
|---|---|---|---|---|
| `barcode_task` | 1 | 16 | none: polls continuously | Reads all three sensors, times GP16 edges, decodes barcodes, publishes `LineBarcodeData_t` |
| `line_follow_task` | 0 | 17 | 10 ms (`S3_FOLLOW_PERIOD_MS`) | Steering, junction detection, barcode turns; sends speed commands and goals to S2 |

The barcode task gets its own core so that bar widths are timed without being delayed by other tasks.

S3 only drives when `line_follow_allowed()` is true:
- the operator has sent START (S1);
- S5 is monitoring with no stop request;
- the S2 brake is not latched;
- S4 has finished calibrating.

Otherwise S3 stops the wheels and waits.

### 2.2 Sensor logic and steering

| GP2 (left) | GP4 (centre) | GP16 (right) | Position | Wheel command (left / right, mm/s) |
|:-:|:-:|:-:|---|---|
| any | 1 | any | Centred (both line sensors on black = junction, §4) | 45 / 45 (`S3_LINE_SPEED_MM_S`) |
| 1 | 0 | any | Line to the left | 30 / 70: a gentle arc (~16 cm radius) |
| 0 | 0 | 1 | Line bent right, under the barcode sensor | 70 / 30 |
| 0 | 0 | 0 | Lost | Search: the guessed side for 300 ms, then the other side |

Refinements, each from a floor problem:

| Rule | Constant | Why |
|---|---|---|
| **Hard turn:** after steering the same way for 400 ms, the inner wheel stops (pivot about it) | `S3_STEER_HARD_MS` = 400 | Curves tighter than the gentle arc |
| **Soft exit:** for 150 ms after the centre finds the line again, the "inner" wheel runs at 50 mm/s and the outer at 45 (see §6.5) | `S3_STEER_EXIT_MS` = 150 | Snapping straight at once caused a straight / steer / straight zig-zag on curves |
| **Hold straight while a barcode is being scanned** (centre still on the line), at most 4.5 s | `S3_SCAN_HOLD_MAX_MS` = 4500 | Steering changes GP16's speed in mid-scan and stretches bars |
| **Line lost:** stop after 2.5 s of searching | `S3_LINE_LOST_MS` = 2500 | The car must not wander off the course |
| **After an S5 bypass:** start following straight away; the search finds the line if the car stopped just off it | — | 20 Nov: S3 used to wait for a sensor on the line and the car stood still |

## 3. Barcode decoder

Code 39: every character is 9 elements (5 bars, 4 spaces), 3 of them wide. A printed barcode is `*` letter `*` with narrow gaps, so a full scan is **29 elements**.

1. **Timing.** Every GP16 level change closes an element. Its duration is stored in 8 µs units (`uint16_t`, 60 entries, rolling; CR1). Pulses under 100 µs are noise and are dropped.
2. **End of scan.** No edge for 400 ms (`TIMEOUT_US`) ends the scan. That is longer than the widest bar takes at the slowest steering speed (9 mm at 45 mm/s = 200 ms). A scan of fewer than 5 elements, for example the track line crossing GP16, is ignored.
3. **Frame search.** At every offset, the decoder classifies elements 0–8 and 20–28. It accepts the offset only if **both** read as `*`, forwards or mirrored. A letter is never accepted without its frame, because nine durations taken at the wrong offset can look like another letter. The mirror test gives the scan direction, which separates the six mirror-image pairs (K/U, N/Y, O/X, Q/W, R/V, T/Z).
4. **Classifying nine elements** (`classify_nine()`):
   - **Bar-widening correction.** The IR spot sees black early and leaves it late, so bars read wider and spaces narrower by the same amount. Every character has 3 narrow bars and 3 narrow spaces, so half the difference between the middle narrow bar and the middle narrow space is subtracted from each bar and added to each space. Without this, 1.5 mm of widening already failed most simulated scans.
   - **Threshold.** Sort the nine values. The threshold is midway between the 6th and 7th. They must differ by at least 1.3 : 1, and exactly three elements must be wide.
5. **Lookup.** The middle nine (elements 10–18), mirrored back if needed, are matched exactly against the 43-entry table. `*` itself is never returned as a letter.
6. **Result.** A decoded letter goes to `mbf_barcode` (IF-01) and into `LineBarcodeData_t`. A second decode within 300 ms is ignored (`DECODE_COOLDOWN_US`). Every finished scan also reports one of four results, which S1 sends to the dashboard so that a failed read can be diagnosed:

   | Result | Meaning |
   |---|---|
   | `OK` | A letter was decoded |
   | `SHORT` | Fewer than 29 elements |
   | `NO_FRAME` | No `*` … `*` frame at any offset |
   | `BAD_LETTER` | Framed, but the middle nine are not a letter |

## 4. Navigation command interface

### 4.1 From letter to movement

```
MQTT "map CDAB"  ──S1──►  set_mqtt_action_mapping()     (IF-17, one 32-bit store)
GP16 scan  ──►  decoder  ──►  mbf_barcode 'A'           (IF-01)
                                   │
line_follow_task: stores 'A' ──► next junction (GP2 + GP4 black ≥ 40 ms)
                                   │
              get_action_for_character('A') = MOVE_LEFT
                                   │
     S2 goal: forward 60 mm (wheels onto the junction) ──► pivot left,
     counting lines under GP4 ──► line found ──► line following resumes
```

| Movement | `MovementAction_t` | At the junction |
|---|---|---|
| Turn left | `MOVE_LEFT` | Advance 60 mm, pivot left, stop on the first line |
| Turn right | `MOVE_RIGHT` | Advance 60 mm, pivot right, stop on the first line |
| U-turn | `MOVE_UTURN` | Advance 60 mm, pivot left, pass the left branch, stop on the second line (the one the car came in on) |
| Go straight | `MOVE_FORWARD` | Keep following |
| No barcode, or a letter not in the map | `MOVE_NONE` | Keep following (S1 reports INVALID for an unmapped letter) |

- **The map is set over MQTT:** four different letters or digits for forward, U-turn, left and right. For example, the Write-up's example table (A left, B right, C straight, D U-turn) is `map CDAB`.
- **Send the map after every boot or reconnect.** The car keeps no map across a reboot, and until a map arrives every barcode means "straight on".
- **One action per junction.** The stored letter is cleared when it is used, and the junction re-arms only after the car has left it.
- **Turn safeguards.** The pivot ignores lines until the encoders show at least 25° of turn (`S3_TURN_MIN_DEG`). It gives up after 6 s and falls back to the normal line search.
- **Why turns use the sensors, not an encoder angle.** On 19 Nov the provisional wheel calibration turned ~1.7× too far: a U-turn went 270° and a right turn 180°. Counting lines under GP4 does not depend on that calibration.

### 4.2 Interfaces to other subsystems

| ID | Direction | Call / object | Content |
|---|---|---|---|
| IF-01 | S3 → S3 | `mbf_barcode` (message buffer, 1 B) | Decoded letter, barcode task → line-follow task |
| IF-08 | S3 → S5 | `get_line_barcode_data_safe()` | Line sensors for S5's line recovery after a bypass |
| IF-10 | S3 → S1 | `get_line_barcode_data_safe()` | Sensor states, newest letter, decode counter, scan result (telemetry) |
| IF-17 | S1 → S3 | `set_mqtt_action_mapping(fwd, uturn, left, right)` | Barcode map from the MQTT `map` command |
| — | S3 → S5 | `s3_line_following()` | TRUE while S3 is following, so S5 knows when the car is on the line |
| — | S3 → S2 | `b2_submit_command()`, `b2_goal_start()` / `b2_goal_check()`, `b2_stop()` | Wheel speeds while following; the 60 mm advance before a turn |

All shared data is copied under S3's kernel mutex. The map is one aligned 32-bit word, so a reader never sees half an old and half a new mapping (CR1).

## 5. Evidence

| Test | What it runs | Result (26 Nov 2026, commit `5901132`) |
|---|---|---|
| S3-H01 decoder ([`s3_driver.c`](../../testing/cr1/host_test/s3_driver.c)) | Real `line_barcode.c`. 3,000 random Code 39 scans: random letter, speed and narrow:wide ratio, ±25 % jitter, both directions, damaged scans | 2,914 correct, **0 wrong**; all 86 failures reported as "seen, not decoded" (19 no frame, 67 bad letter) |
| S3-H02 junction turns ([`s3_junction_sim.c`](../../testing/cr1/host_test/s3_junction_sim.c)) | Real `line_barcode.c` with line following on, driving a simulated car over a cross junction. Sensor offset 40 / 60 / 80 mm, sensor spacing, encoder and turn errors (×0.8 … ×1.7), slow acceleration, start offset | **40 / 41**: L, R, U, F and an unmapped letter all correct in 8 car variants. The one failure is §6.1 |
| S3-H03 decode while following (same simulator) | 14 variants × 20 random letters: bars ±1.4 mm, GP16 35–70 mm right of the line, wheel-speed noise 10 / 20 %, weaving start | 279 / 280 decoded, **0 wrong** (1 not decoded at 20 % noise) |
| S3-M01 floor, 13 Nov | Real barcode on the course | One scan INVALID, then LEFT decoded correctly and BARCODE_NAV entered |
| INT-06 floor, 13 Nov | Line following on the course | Longest unbroken stretch 51.8 s; 102 stretches, median 0.60 s |

Run all of the host tests with `sh INF2004_Project/testing/cr1/host_test/run_host_tests.sh`.

## 6. Known limitations

### 6.1 A sharp left curve between a barcode and its junction triggers the turn early

A junction is "both line sensors on black". A sharp left curve (or a short stub) also puts GP2 and GP4 on black together. If one lies between the barcode and the real junction, the stored action runs there. In the simulator, barcode R at a course with a left stub gives a U-turn at the stub instead of a right turn at the junction (S3-H02, the one failure).

- **On the course:** check that no sharp left bend sits between a barcode and the junction it belongs to.
- **Possible fixes, to test on the car first:**
  - require the junction to last a minimum travelled distance (a cross is wider than a curve);
  - after the 60 mm advance, check that the line also continues straight ahead before pivoting.

### 6.2 Coarse line position

Two digital line sensors give only "centred / left / right / lost", so steering is bang-bang. That explains the short follow stretches on the floor (median 0.6 s). Line following runs at 45 mm/s to keep this stable. Analogue sensors, or a third line sensor on the right (GP16 is busy with barcodes), would allow proportional steering.

### 6.3 No software calibration

The black/white threshold is set by each board's trimmer pot. If the lighting or the surface changes, re-set the pots on the course before the run.

### 6.4 On-target self-tests not logged

S3-U01 … U04 exist in `line_barcode_test.c` (`make TEST=1`) but have no result row yet.

### 6.5 The soft exit arcs slightly the wrong way

The code comment says the soft exit keeps "a gentle arc the same way". But the inner wheel runs at `S3_STEER_EXIT_INNER_MM_S` = 50 mm/s while the outer wheel runs at `S3_LINE_SPEED_MM_S` = 45 mm/s, so the inner wheel is the faster one. The car therefore turns slightly *away* from the curve for 150 ms. The 50 probably dates from a faster line speed (it was 70 and 100 earlier); it became active on 20 Nov, when the soft exit was switched on.

The effect is small (5 mm/s for 150 ms). If the intent is an arc the same way, set the inner speed below 45 (e.g. 35) and check it on the S-curves before the demo. No logged floor run since 20 Nov shows how it behaves either way.
