# Change Requirement 1: Minimum-Bit Data Representation — Report

Week 13 evidence for Change Requirement 1. Branch `cr1/min-bit`; the "before" build is git tag `pre-cr1` (commit `18b1bd7`).

| Deliverable | Where |
|---|---|
| Data representation table | [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md) |
| RAM usage comparison | §1 below; raw data in [BASELINE.md](BASELINE.md), [baseline_ram_symbols.txt](baseline_ram_symbols.txt), [after_ram_symbols.txt](after_ram_symbols.txt) |
| Message size comparison | §2 below |
| Correctness and performance | §3 and §4 below; tests in [host_test/](host_test/) |

## Summary

- **Application RAM: 1,589 → 1,138 bytes (−451 B, −28 %).** Every subsystem went down. (1,122 B at the CR1 commit; +16 B on 19 Nov for B2's per-wheel calibration and straight-line correction, see the note under §1.)
- **Inter-task messages are 25–50 % smaller.** For example, the S5 obstacle event went from 32 to 20 B and the S2 command mailbox from 16 to 8 B.
- **MQTT payloads are compact binary.** A status frame is 21 B, against 257 B for the same data as JSON text (8 %). This was already true before the change (Buddy 1's codec), so the wire format is unchanged.
- **One system-wide fault bitmask** gives each fault its own bit: motor stall, IMU fault, no echo, Wi-Fi loss and so on.
- **No loss of functionality on the host.** For S3, S4 and S5, the pre-CR1 and CR1 code were run side by side on the same simulated sensor input and gave identical results at the CR1 commit. (A 23 Nov re-run shows S3 decoding 0.4 % fewer simulated scans after later, non-CR1 barcode changes, still with no wrong letters; see §3.) S1 and S2 have new unit tests. In total over 200,000 simulated cases and about 2,100 unit checks pass.
- **Timing on the car:** the pre-CR1 baseline (13 and 16 Nov floor logs) and the before/after measurements are in §4.

## 1. RAM usage

Measured with `arm-none-eabi-size` / `nm` on the demo build (`make NET=lwip MQTT=1`). Each figure is the static `.data` + `.bss` of the subsystem's object files (test files excluded).

| Subsystem | Before (B) | After (B) | Change |
|---|---:|---:|---:|
| S1 Telemetry / MQTT | 409 | 378 | −31 (−8 %) |
| S2 Motion | 263 | 158 ¹ | −105 (−40 %) |
| S3 Line + barcode | 316 | 167 | −149 (−47 %) |
| S4 IMU / terrain | 278 | 206 | −72 (−26 %) |
| S5 Ultrasonic / servo | 323 | 229 | −94 (−29 %) |
| **Application total** | **1,589** | **1,138** ¹ | **−451 (−28 %)** |

S1's remaining 378 B is mostly the 289 B raw MQTT receive buffer, which is a byte stream with nothing to pack.

| Whole image | Before | After | Change |
|---|---:|---:|---:|
| RAM (`data` + `bss`) | 84,256 | 83,780 ¹ | −476 |
| Flash (`text`) | 369,349 | 372,437 ¹ | +3,088 (+0.8 %) |

¹ Re-measured on 19 Nov after B2's task-list additions (branch `buddy2/motion-tasks`). At the CR1 commit S2 was 142 B, the application 1,122 B, the image 83,768 B RAM / 371,729 B flash. The additions are new functions, not a loss of packing: the goal tick count became one per wheel (+4 B, each wheel can have its own ticks per turn), the straight-line correction keeps the encoder counts at the start of a straight (+8 B, full 32-bit words like the ISR counters they are subtracted from), and the D term keeps last loop's speed per wheel (+4 B, two `int16_t`: two 10-bit fields would also take 4 B once aligned). Rows in [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md), S2.

Most of the whole-image RAM belongs to the network stack, the Wi-Fi driver and the kernel, which are outside this change. Flash grows slightly because reading and writing bit-fields takes extra shift and mask instructions. Trading a little flash for RAM is the expected cost of this change.

Kernel memory also shrinks. The S5 event queue is sized from `sizeof(subsystem5_event_t)`, so it drops from 320 to 224 B.

### Largest single savings

| Data | Before | After |
|---|---:|---:|
| S3 barcode bar widths (60 entries) | 240 B (`uint32_t` µs) | 120 B (`uint16_t`, 8 µs units) |
| S5 scan samples (20 rays) | 122 B | 84 B |
| S5 status + snapshot | 2 × 38 B | 2 × 20 B |
| S4 status | 56 B | 32 B |
| S2 status | 48 B | 28 B |
| S2 / S4 calibration configs | 2 × 18 B RAM copies each | `const`, in flash |

## 2. Message sizes

### Inter-task messages and shared snapshots (ARM `sizeof`)

| Message / shared data | Path | Before | After |
|---|---|---:|---:|
| `subsystem5_event_t` | S5 → VC message buffer (IF-02) | 32 | **20** |
| S2 command mailbox | S3/S5/VC → S2 (IF-03) | 16 | **8** |
| `b2_event_t` | S2 events | 2 | **1** |
| `b4_event_t` | S4 events | 12 | **8** |
| `b2_status_t` | S2 → all (snapshot) | 48 | **28** |
| `b4_status_t` | S4 → S1, VC, S3 | 56 | **32** |
| `subsystem5_status_t` | S5 → S1, VC, S3 | 38 | **20** |
| `LineBarcodeData_t` | S3 → S1, VC | 6 | **4** |
| `vehicle_status_t` / `motion_status_t` / `line_status_t` / `imu_status_t` / `ultra_status_t` | team getters → S1 | 4 / 16 / 4 / 18 / 14 | **2 / 8 / 3 / 10 / 8** (line: 2 B packed + 1 B for the barcode letter, added 19 Nov) |
| `S1_STATS` | Wi-Fi task → telemetry task | 20 | **8** |
| `S1_CMD` | Wi-Fi task → telemetry task | 12 | 12 (see table, "left as is") |
| `vehicle_event_t` | `qVehicleEvent` | 8 | 8 (already minimal once aligned) |
| barcode letter | `mbf_barcode` | 1 | 1 (smallest message the kernel supports) |

### MQTT payloads

Every frame is bit-packed, most-significant bit first ([tlm_codec.h](../../app_program/subsystem1/tlm_codec.h)). The field widths are listed in the data representation table.

| Frame | Bits | Bytes | Same data as JSON text |
|---|---:|---:|---:|
| Status (5 Hz) | 166 | **21** | 257 B typical, 276 B worst case |
| Heartbeat (1 Hz) | 86 | 11 | – |
| Event (max) | 78 | 10 | – |
| BARCODE event | 35 → 41 | 5 → 6 | – (19 Nov: now also carries the decoded letter as a 6-bit code, and reports "seen, not decoded" scans) |
| ACK | 52 | 7 | – |
| Command / `map` command | 40 / **64** (was 72) | 5 / **8** (was 9) | – (19 Nov: map letters as 6-bit codes instead of 8-bit ASCII) |

The JSON figure comes from the codec's own `tlm_format_status_json()` on the same status values ([json_vs_binary.c](host_test/json_vs_binary.c)). At 5 Hz, binary status costs 105 B/s against about 1,285 B/s as text.

## 3. Correctness: no loss of functionality

All tests run on a laptop with one command:

```sh
sh INF2004_Project/testing/cr1/host_test/run_host_tests.sh
```

The script rebuilds the pre-CR1 code from tag `pre-cr1` automatically, so each old-vs-new comparison runs the real old source against the real new source.

| Subsystem | Test | Result |
|---|---|---|
| S1 | Existing codec test (wire format unchanged) | PASS |
| S1 | [test_s1_adapters.c](host_test/test_s1_adapters.c): real `team_adapters.c`. Every fault sets exactly its own system-mask bit, the MQTT fault byte follows the pre-CR1 rules, values saturate at their field width instead of wrapping, barcode letters map to the right movement, and the letter and "seen, not decoded" count reach S1 | 109/109 checks (112/112 on the 23 Nov re-run) |
| S2 | [test_s2_motion.c](host_test/test_s2_motion.c): real `motion.c` with a simulated clock and encoder ISR. Covers packing at every range edge, refusing out-of-range requests (never truncating), MOVE goal distance and speed, the 32-bit clock wrapping mid-run, stall fault and reset, emergency brake latch, odometry saturation, and every lock released. Since 19 Nov also B2's additions on a simple motor model (deadband, lag, 10 % weaker right motor): straight-line correction (27 → 1 tick of drift over 5 s), per-wheel calibration, D term stability, the move/turn API, and S5's tick constants unchanged | 1,974/1,974 checks (1,093 at the CR1 commit) |
| S3 | [s3_driver.c](host_test/s3_driver.c): old vs new `line_barcode.c` on 3,000 simulated Code 39 scans (random letter, speed, narrow:wide ratio, ±25 % jitter, both directions, damaged scans), plus the barcode map | At the CR1 commit: **identical**, 2,927 decoded, all correct, none wrong; all 73 undecoded scans reported as "seen, not decoded". **23 Nov re-run** (after the post-CR1 barcode scan-diagnosis changes, `85cd8b2`): 2,914 correct (pre-CR1 2,927, −0.4 %), **0 wrong**; all 86 undecoded scans reported (19 no frame, 67 bad letter). The difference comes after CR1 and is not from the bit packing; it is listed for transparency |
| S1 + dashboard | [barcode_event_enc.c](host_test/barcode_event_enc.c) + [test_barcode_event.py](host_test/test_barcode_event.py): every letter A–Z, 0–9, plus "seen, not decoded" and INVALID, encoded by the C codec and decoded by the real `dashboard.py`; old 5-byte events still decode | 82/82 checks (177/177 on the 23 Nov re-run, after the 7-byte BARCODE event) |
| S4 | [s4_driver.c](host_test/s4_driver.c): old vs new `imu.c`/`terrain.c`/`posture.c` on 12 simulated runs: calibrate, drive, hump, turn, **lifted → motors stopped**, reset, **overturned → motors stopped**, I2C fault | **Identical**: 2,074 status/event lines, 12 humps, 24 lift/overturn stops, locks balanced |
| S5 | [test_s5_profile.c](host_test/test_s5_profile.c): old vs new `profile.c` on 200,000 random scans | **Identical**: 0 mismatches over 16 fields |

Both the demo build (`NET=lwip MQTT=1`) and the test build (`TEST=1`) compile with no warnings. The on-target self-tests (S2-U01..U06, S4-U01..U16 and so on) are unchanged and still need to be run on the car.

### Concurrency (packed data shared between tasks / ISR)

A bit-field write is a read-modify-write of the whole storage unit, so every packed field that is shared between tasks was checked.

| Shared data | Rule applied |
|---|---|
| S2 encoder ISR counters and edge times | Kept as plain aligned 32-bit words: the ISR does one store and never touches packed data. The edge time went from 64 to 32 bits, which **removes** a two-store ISR write that a reader could catch half-done. |
| S2 flags | Split by writer. Flags that other tasks write are only ever written under `s_lock`. **Fixed:** two of them were previously cleared without the lock. |
| S4 status control byte (state, faults, lift, motion) | Every write now holds `s_lock`, including in the 100 Hz poll. **Fixed:** the remote RESET (S1's task) and the poll used to write these fields with no common lock. |
| S3 MQTT barcode map | Four letters packed into one 32-bit word, written and read in one access. **Fixed:** a reader could previously see half an old and half a new mapping. |
| S3 / S5 / line-follow task flags | Single-writer bit-field structs (only the owning task writes). Readers on other tasks only read. |
| S5 command flags set by other tasks | Deliberately **left one byte each**: a byte store needs no lock, while a shared bit-field byte would. |
| S1 team structs, `S1_STATS` | One writer each: the telemetry task and the Wi-Fi task respectively. |
| System fault bitmask | Built on demand from each subsystem's own (locked) fault mask, so it has no writer of its own. |

## 4. Performance

### Expected effect (from code inspection)

- **Encoder ISR:** slightly cheaper (one 32-bit timestamp store instead of two).
- **S2 control loop:** one extra short critical section per cycle for the stop-event check.
- **S4 100 Hz loop:** at most two or three extra short critical sections per sample (the terrain phase every sample, the motion class only when it changes).
- **Bit-field access:** a few extra instructions per access, which is the +0.6 % flash.
- **Algorithms:** no thresholds, filters or tuned constants were changed.

### Measured: pre-CR1 baseline from the floor logs

All dashboard logs in `testing/subsystem1/logs/` were captured before CR1 was merged (18 Nov), so they are the "before" side. Figures from `testing/tools/analyse_logs.py` (re-run 23 Nov):

| Measurement | 13 Nov (floor, line following) | 16 Nov (floor, groupmate's log) | Target |
|---|---:|---:|---|
| Status frame interval, median | 195 ms | 202 ms | 200 ms (5 Hz) |
| Status frame interval, p95 | 512 ms | 349 ms | |
| Heartbeat interval, median | 1,017 ms | 964 ms | 1,000 ms |
| Status frames lost (sequence gaps) | 9.6 % | 1.5 % | |
| Command ACK round trip, median | 397 ms | 182 ms | |

The losses and the long tails line up with Wi-Fi reconnects (5 and 4 in those sessions), not with CPU load.

### After CR1: on the car

| # | Measurement | Before (`pre-cr1`) | After (CR1) | How |
|---|---|---|---|---|
| T1 | S2 control-loop deadline misses | | | `deadline_misses` in `b2_status_t`; test S1-U17 checks it does not grow under telemetry load |
| T2 | Telemetry task worst-case loop time | | | `S1_DIAG.loop_max_us` (`s1_tlm_get_diag()`, printed by the S1 tests) |
| T3 | Status frame interval, median (5 Hz target = 200 ms) | 195 ms (13 Nov) / 202 ms (16 Nov) | | Dashboard CSV over a 5-minute run |
| T4 | Lift → motors stopped | | | S4-M01: pick the car up while it drives; it must stop at once and stay stopped until RESET |
| T5 | Full course run | | | Line, barcode, hump, obstacle: compare telemetry logs before/after |

Results also go in the TEST_LOGs under `testing/`.

### Why no degradation is expected

- **Wire load is unchanged.** The STATUS (21 B) and HEARTBEAT frames are the same size before and after. Only the BARCODE event (6 → 7 B, carries the letter) and the MAP command (9 → 8 B) changed, and both are rare. So the network side, which dominates the timings above, sees the same traffic.
- **CPU cost is a few instructions per field access** (+0.8 % flash, §1). On a 125 MHz RP2040 with 20 ms (S2) and 10 ms (S4) loops, that is far below one loop period.
- **Behaviour is identical on the host** for S4 and S5 and equivalent for S3 (§3). No threshold, filter or gain was changed.

## 5. Commits

| Commit | Content |
|---|---|
| `1bce2df` | Baseline measurements, data representation table, IMU docs |
| `5eb0e76` | S2 motion |
| `55ab5e4` | S5 ultrasonic |
| `86bf1b0` | S3 line + barcode |
| `57db409` | S4 IMU |
| `df11bab` | S1 + integration, system fault bitmask |

Each subsystem is a separate commit, so any one of them can be reverted on its own if the car shows a problem.
