# IS33 MQTT Protocol (Buddy 1: WiFi Communication, Command and Telemetry)

This document describes the MQTT link between the robot car and the laptop dashboard: topics, message formats, command rules and connection recovery. It is the "MQTT documentation" deliverable from the Project Write-up.

| Item | Where in the code |
|---|---|
| Topics, rates, buffer sizes | [tlm_config.h](tlm_config.h) |
| Frame encoders, command validator | [tlm_codec.h](tlm_codec.h), [tlm_codec.c](tlm_codec.c) |
| Sessions, ACKs, counters (runs in the WiFi task) | [tlm_bridge.c](tlm_bridge.c) |
| Connect, last will, subscribe, reconnect | `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` |
| Telemetry task (builds frames, runs commands) | [task_telemetry.c](task_telemetry.c), [team_adapters.c](team_adapters.c) |
| Laptop dashboard (decoder, commands, CSV log) | `testing/subsystem1/tools/dashboard.py` |
| Host tests | `testing/subsystem1/host_test/`, `testing/cr1/host_test/` |

**If you change a format, update the C codec, `dashboard.py` and this file together.**

## 1. Setup

| | |
|---|---|
| Broker | Mosquitto on the laptop: `mosquitto -c testing/subsystem1/tools/mosquitto.conf -v` (listens on port 1883 for the whole hotspot, anonymous) |
| Car side | Wi-Fi SSID/password and broker IP/port in `config/wifi_secrets.h` (copy from `wifi_secrets.example.h`; not for sharing) |
| Firmware | `cd build_make && make NET=lwip MQTT=1 -j8` → `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2` |
| Dashboard | `python3 testing/subsystem1/tools/dashboard.py --broker <laptop IP>` (needs `paho-mqtt`). Options: `--port`, `--log tlm_log.csv`, `--no-panel`, `--quiet`, `--clear-retained` |
| Keepalive | 5 s; the broker declares the car gone after about 7.5 s of silence |

On the MQTT build the car **waits for `start`** before it drives (IF-18).

## 2. Topics

| Topic | Direction | QoS | Retained | Rate | Payload |
|---|---|---:|:---:|---|---|
| `is33/car/online` | car → laptop | 1 | yes | on connect; last will on loss | ONLINE frame (3 B) |
| `is33/car/status` | car → laptop | 0 | no | 5 Hz | STATUS frame (21 B) |
| `is33/car/heartbeat` | car → laptop | 0 | no | 1 Hz | HEARTBEAT frame (11 B) |
| `is33/car/event` | car → laptop | 1 | no | on change | EVENT frame (5–10 B) |
| `is33/car/ack` | car → laptop | 1 | no | one per command | ACK frame (7 B) |
| `is33/car/cmd` | laptop → car | 1 (subscribed) | must not be | on demand | COMMAND frame (5 or 9 B) |

Why these choices:
- **QoS 0 for status and heartbeat:** they repeat every 200 ms / 1 s, so a lost one is replaced by the next.
- **QoS 1 for events, ACKs and commands:** each is a one-off that must arrive. A QoS 1 duplicate is caught by the sequence-number check (§5).
- **`online` is retained:** a dashboard started later still sees whether the car is up, and which session it is using.

## 3. Frame encoding

- **All payloads are binary and bit-packed**, most-significant bit first, with no padding between fields. The last byte is zero-padded. This follows Change Requirement 1 (minimum-bit representation), and `testing/cr1/` has the evidence.
- **Out-of-range values saturate** at the field's maximum instead of wrapping (`bw_put_sat`). Counters marked "wraps" are the exception.
- **Signed values are sent offset into an unsigned range** (pitch + 64, yaw + 256), so no bit operations touch signed data (Barr C).
- **Size:** a STATUS frame is 21 B binary; the same data as JSON text (`tlm_format_status_json()`) is about 257 B.

### 3.1 Common header (28 bits, every frame except ONLINE and COMMAND)

| Field | Bits | Meaning |
|---|---:|---|
| version | 4 | Protocol version, currently **1** |
| session | 16 | Session number of this connection (§5) |
| seq | 8 | Per-topic frame counter, wraps (lets the dashboard spot lost frames) |

### 3.2 ONLINE (`is33/car/online`): 24 bits = 3 B

| Field | Bits | Meaning |
|---|---:|---|
| version | 4 | 1 |
| online | 1 | 1 = online (published on connect), 0 = offline (the last will) |
| reserved | 3 | 0 |
| session | 16 | Session of this connection. Commands must carry it |

### 3.3 STATUS (`is33/car/status`, 5 Hz): 166 bits = 21 B

Fields in wire order, after the 28-bit header:

| # | Field | Bits | Range / encoding |
|---:|---|---:|---|
| 1 | vehicle_state | 3 | §4.1 |
| 2 | estop_latched | 1 | 1 = S2 brake latched (E-STOP, lift/overturn, fault) |
| 3 | line_state | 2 | `(left << 1) \| centre`: 0 lost, 1 centre, 2 left only, 3 junction |
| 4 | last_barcode | 3 | §4.2 |
| 5 | speed_l | 10 | mm/s, 0..1023, saturates (direction not sent) |
| 6 | speed_r | 10 | mm/s, as above |
| 7 | enc_l | 16 | Encoder edges, wraps |
| 8 | enc_r | 16 | Encoder edges, wraps |
| 9 | distance | 12 | cm, 0..4095, saturates |
| 10 | pitch | 7 | Degrees, sent as pitch + 64 (−64..63) |
| 11 | hump_max | 8 | mm, highest hump this run (0..255) |
| 12 | obstacle_state | 3 | §4.4 |
| 13 | obstacle_distance | 9 | cm, 0..510; 511 = no echo |
| 14 | fault_mask | 8 | Bit n = subsystem n has a fault (bits 2..5) |
| 15 | yaw_rate | 9 | °/s, sent as yaw + 256 (−256..255), + = turning left |
| 16 | motion | 3 | §4.5 |
| 17 | up_speed | 8 | Upward speed / 4 (0..1020 mm/s), lift signal |
| 18 | tilt | 7 | Degrees from level, 0..127, saturates |
| 19 | s5_fault | 3 | §4.6 |

Fields 15–19 were appended later. A decoder that finds fewer bits (an older 17- or 19-byte frame) skips them; `dashboard.py` does this.

### 3.4 HEARTBEAT (`is33/car/heartbeat`, 1 Hz): 86 bits = 11 B

| Field | Bits | Meaning |
|---|---:|---|
| uptime | 16 | Seconds since boot, saturates (~18 h) |
| rssi | 7 | −dBm (0 = not available to app tasks; currently always 0) |
| link_state | 3 | §4.7 (always ONLINE, because it is only sent while online) |
| cmds_ok | 8 | Commands accepted, wraps |
| cmds_rejected | 8 | Commands rejected, wraps |
| queue_full | 8 | Valid commands dropped because the queue was full, wraps |
| reconnects | 8 | MQTT reconnections since boot, wraps |

### 3.5 EVENT (`is33/car/event`, QoS 1): 4-bit type, then a type-specific payload

| Type | Value | Payload | Total |
|---|---:|---|---|
| STATE_CHANGE | 1 | from state (3), to state (3) | 38 bits, 5 B |
| BARCODE | 2 | action (3, §4.2), **letter (6)**: 0 none, 1–26 = A–Z, 27–36 = 0–9. Action NONE + letter 0 = **barcode seen but not decoded** | 41 bits, 6 B |
| HUMP | 3 | height of the completed hump, mm (8) | 40 bits, 5 B |
| OBSTACLE | 4 | nearest cm (9), centre angle ° (8), width cm (8), left clearance cm (9), right clearance cm (9), chosen side (3: 0 none, 1 left, 2 right) | 78 bits, 10 B |
| LIFTED | 5 | cause (2): 1 lifted, 2 overturned | 34 bits, 5 B |
| ESTOP | 6 | source (3, §4.8) | 35 bits, 5 B |

When each event is sent: on a change seen by the 5 Hz status task.
- **STATE_CHANGE:** the vehicle state changed.
- **ESTOP:** the brake latch went from clear to set.
- **BARCODE:** S3's decode counter changed (or its "seen, not decoded" counter).
- **HUMP:** S4's hump counter changed.
- **LIFTED:** S4 latched a lift or overturn.
- **OBSTACLE:** S5 measured a new obstacle profile.

**Older BARCODE events:** firmware before 19 Nov 2026 sent BARCODE without the letter (5 B). Those frames have only 5 padding bits after the action, so a decoder that needs 6 more bits for a letter correctly finds none.

### 3.6 ACK (`is33/car/ack`, one per command): 52 bits = 7 B

| Field | Bits | Meaning |
|---|---:|---|
| cmd_seq | 16 | The `seq` of the command being answered |
| cmd | 4 | Its command code (§3.7) |
| result | 4 | §4.9 |

The ACK says the command was **accepted and queued**, not that it has been carried out. The telemetry task carries it out within about 50 ms; the result shows in STATUS and on the console.

### 3.7 COMMAND (`is33/car/cmd`, laptop → car): 40 bits = 5 B, or 72 bits = 9 B for `map`

| Field | Bits | Meaning |
|---|---:|---|
| version | 4 | 1 |
| code | 4 | 1 START, 2 STOP, 3 E-STOP, 4 RESET, 5 MAP |
| session | 16 | Must equal the session in the car's current ONLINE message |
| seq | 16 | Must be higher than the last accepted seq of this session |
| map letters | 4 × 8 | **MAP only**: four different ASCII letters/digits (`A–Z`, `0–9`) for forward, U-turn, left, right |

## 4. Value tables

### 4.1 vehicle_state
0 INIT · 1 IDLE · 2 LINE_FOLLOW · 3 LINE_SEARCH · 4 BARCODE_NAV · 5 OBSTACLE_AVOID · 6 SAFE_STOP

### 4.2 Barcode action (`last_barcode`, BARCODE event)
0 none · 1 LEFT · 2 RIGHT · 3 STRAIGHT · 4 U-TURN · 5 INVALID (letter decoded but not in the map)

### 4.3 fault_mask (STATUS byte)
bit 2 S2 motors/encoders · bit 3 S3 (unused: line loss is reported as a line state, not a fault) · bit 4 S4 IMU · bit 5 S5 ultrasonic.

It is derived from the 16-bit system fault bitmask (`SYS_FLT_*` in `team_interfaces.h`), which has one bit per fault condition, including Wi-Fi loss. The vehicle controller prints that mask on the console every 5 s.

### 4.4 obstacle_state
0 MONITOR · 1 WAIT_STOP · 2 COARSE scan · 3 FINE scan · 4 PROFILE_READY · 5 BYPASS · 6 FAULT · 7 REVERSE

### 4.5 motion
0 CALIBRATING · 1 STATIONARY · 2 MOVING · 3 TURNING · 4 ON_HUMP · 5 LIFTED · 6 OVERTURNED

### 4.6 s5_fault
0 none · 1 sensor not answering (no echo) · 2 scan invalid · 3 car did not stop · 4 event lost · 5 no safe side · 6 bypass blocked/failed

### 4.7 link_state
0 INIT · 1 WIFI_CONNECTING · 2 MQTT_CONNECTING · 3 ONLINE · 4 BACKOFF · 5 SAFE_LOCAL

### 4.8 E-STOP source
0 button · 1 laptop · 2 IMU (lifted/overturned) · 3 motion · 4 ultrasonic · 5 line

### 4.9 ACK result
| Code | Name | Meaning |
|---:|---|---|
| 0 | ACCEPTED | Valid and queued |
| 1 | BAD_LEN | Not 5 B (or not 9 B for MAP, or 9 B for another command) |
| 2 | BAD_VERSION | Version is not 1 |
| 3 | BAD_SESSION | Wrong session: sent to an old connection, or a retained message |
| 4 | BAD_CMD | Unknown code, or an invalid MAP (bad or repeated letter) |
| 5 | STALE_SEQ | seq not above the last accepted one: a replay or QoS 1 duplicate |
| 6 | QUEUE_FULL | Valid, but four commands were already waiting |

Checks run in this order; the first failure is reported. A rejected command never uses up a sequence number.

## 5. Command rules

1. **Session:** every new connection picks a new, non-zero, random-looking session number. It is published in the retained ONLINE message, and the dashboard reads it from there. While offline, the session is 0 and every command is rejected.
2. **Sequence:** within a session, each accepted command must have a higher `seq` than the one before. The sequence restarts with every new session. The dashboard seeds its first `seq` from the clock, so a newly opened dashboard window gets above an older one.
3. **Effect:** commands from before a dropout, retained commands, replays and QoS 1 duplicates can never act twice or on the wrong connection.
4. **Two tasks:** the WiFi task only validates, ACKs and queues (never waits; queue depth 4). The telemetry task carries the command out, because that may lock other subsystems:

| Command | Effect | Interface |
|---|---|---|
| START | Allows driving (`s1_mission_go()` = true). Refused while in SAFE_STOP | IF-18 |
| STOP | Stops the wheels; line following waits; S5 cancels a scan/bypass in progress | IF-15, IF-18 |
| E-STOP | `b2_emergency_brake()`: motors off, **latched** until RESET | IF-15 |
| RESET | Only while the wheels are not being driven: clears the S2 brake latch, S4's lift/overturn latch (S4 recalibrates) and S5's fault. Never restarts driving: send START | IF-15, IF-16 |
| MAP xxxx | Sets the barcode letter → movement table (forward, U-turn, left, right), e.g. `map CDAB` = A Left, B Right, C Straight, D U-turn. Until a MAP arrives, every barcode means straight on. The car forgets it on reboot | IF-17 |

## 6. Connection recovery

| Situation | What happens |
|---|---|
| Connecting | The car registers a **last will**: an ONLINE frame with online = 0 and the new session, retained, QoS 1 |
| Connected | The car discards frames queued while offline (stale), publishes ONLINE (online = 1, retained), subscribes to `is33/car/cmd` (QoS 1), and starts accepting commands |
| Link lost (Wi-Fi down, broker gone, car out of range) | About 7.5 s without keepalive and the broker publishes the will, so the dashboard shows the car offline. On the car, the session becomes 0 (commands rejected), the telemetry task produces no frames until it is back, and the system fault mask gets the **Wi-Fi loss** bit. Events that happen while offline are not replayed later; STATUS shows the current state again within 200 ms of reconnecting. **The car keeps driving autonomously; the link is not needed to drive.** |
| Reconnect | Retry every 3 s (not in a tight loop), with a **new session**. `reconnects` in the heartbeat goes up. The dashboard reads the new session from ONLINE automatically, and it also forgets the map it knew, so send `map` again |
| Car reboots | Same as reconnect. The barcode map is lost, so send `map` again |

## 7. Dashboard commands

Type these at the bottom of `dashboard.py`:

| Command | Sends |
|---|---|
| `start` / `stop` / `reset` | START / STOP / RESET |
| `e` or `estop` | E-STOP |
| `map FULR` | MAP with 4 different letters/digits |
| `help` | Field and barcode-letter help |
| `q` | Quit |
| `stale`, `replay`, `burst [n]` | **Tests only:** wrong session, repeated seq, n commands at once (to see BAD_SESSION, STALE_SEQ, QUEUE_FULL) |

Every received frame is decoded, shown in the live panel and appended to the CSV log (`--log`). On demo day the dashboard is the telemetry demonstration.

## 8. Tests

| Test | What it covers |
|---|---|
| `testing/subsystem1/host_test/test_tlm_codec.c` (laptop) | Encoders, saturation, command validator (all ACK results) |
| `testing/cr1/host_test/run_host_tests.sh` (laptop) | Adds: BARCODE events decoded by the real `dashboard.py`, system fault bitmask, team adapters |
| `subsystem1_test.c` (on the car, `make TEST=1 NET=lwip MQTT=1`) | S1-U01..U18: buffers, command handling, priorities, deadline misses, loop time |
| `testing/subsystem1/TEST_LOG.md` | S1-M01..M06: manual dashboard checks, 5-minute soak |
| `testing/subsystem1/2026-11-08_bench_test_results.md` | 8 Nov bench test, 9/9 passed |
