# Dashboard and bench testing: chat log (2026-11-04)

My Q&A with Claude Code while bringing up the MQTT dashboard and bench-testing the car.
Answers are condensed. Wi-Fi details are left out.

## Where things stand (end of session)

| Area | Status |
|---|---|
| Broker + dashboard + MQTT link | Working. Car connects, status/heartbeat arrive, commands get ACKs. |
| Ultrasonic (S5) | **Faulting.** `obs_state: 6`, `obs_cm: 511`, `faults: 32` shortly after boot, and about 8 s after each `reset`. GP1 (ECHO) was wired **without the 5 V to 3.3 V divider**, so GP1 may be damaged. |
| Left encoder (GP6) | **Never counts** (`enc_l` 0 in every session). Right encoder (GP7) counts. |
| Encoder calibration | Firmware assumes **20 ticks per wheel turn** (`subsystem2/tkernel_adapter.c:16`). Our motors have built-in encoders, probably far more, so `speed_r` hits 1023. Needs measuring. |
| Left line sensor (GP2) | Unreliable. `line: 2` seen in two sessions, then nothing; its LED doesn't change over the line. |
| Tilt (S4) | Sensor alive, but `pitch` only blips when tilting nose up. Likely the module's X axis isn't facing forward, or it's loose. |
| `vehicle_state` stuck at 6 | Caused by the ultrasonic fault, not the tilt or the dashboard. |

---

## 1. Starting the dashboard

**Q: How do I start the dashboard? What do I need installed?**

- Python 3 (`winget install Python.Python.3.12`, then reopen the terminal).
- `py -m pip install paho-mqtt`.
- Mosquitto (was already installed).
- Firewall rule for port 1883, run once in an **Administrator** PowerShell (normal PowerShell gives "Access is denied"):
  ```powershell
  New-NetFirewallRule -DisplayName "Mosquitto 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow
  ```
- `MQTT_BROKER_IP` in `config/wifi_secrets.h` must be the laptop's Wi-Fi IP (check with `ipconfig`). The car runs the `NET=lwip MQTT=1` image.

**Q: Where do I run it?**

Two normal PowerShell windows, both in `...\INF2004_Project\INF2004_Project` (the inner folder).

Window 1, broker (leave running):
```powershell
& "C:\Program Files\Mosquitto\mosquitto.exe" -c testing\subsystem1\tools\mosquitto.conf -v
```

Window 2, dashboard:
```powershell
py testing\subsystem1\tools\dashboard.py --broker 127.0.0.1 --quiet --log testing\subsystem1\logs\2026-11-04_dashboard.csv
```

Expect `[dash] connected to broker`, then `[online] ONLINE session=0x....` when the car connects.
Quit with `q`, then Ctrl+C in the broker window.

Optional window 3, live view of the status fields that `--quiet` hides:
```powershell
Get-Content testing\subsystem1\logs\2026-11-04_dashboard.csv -Wait -Tail 1 | Where-Object { $_ -match 'car/status' } | ForEach-Object { $_ -replace '^.*"\{','' -replace '""','' }
```

Notes on the CSV:
- The dashboard appends to an existing file, so use a new name per day.
- Don't open the CSV in Excel while the dashboard runs, because Excel locks it.

---

## 2. Bench test procedure

Keep the wheels lifted. `e` + Enter in the dashboard stops the car at any time.

**Things learned from the firmware that matter for testing:**
- **Never send `stale` then `replay`.** A rejected command doesn't use up its sequence number, so `replay` after `stale` is **accepted and starts the car**. Do `replay` first, then `stale`.
- `start` with no line under the sensor does nothing (state stays 1). Once `start` has been sent, putting tape under the sensor makes the wheels spin straight away.
- `ACCEPTED` in an ACK only means the car received the command. `start` during SAFE_STOP and `reset` while wheels move are still `ACCEPTED` but refused. Check the live view, not the ACK.
- The barcode reader runs without `start`. A scan ends after 0.4 s with no edge, so slide the card in one steady motion.

### Sensors (enter nothing)
| Do this | Look for |
|---|---|
| Turn left wheel, then right | `enc_l` rises, then `enc_r`; speed above 0 while turning |
| Black tape under left / centre / both / none | `line: 2` / `1` / `3` / `0` |
| Tilt nose up about 20° | `pitch` about 20 |
| Hand 30 cm in front | `obs_cm` about 30 |

Tape: matte black (electrical tape) on white paper, 5-10 mm under the sensors, narrower than the gap between the two sensors.

### Commands (no tape, wheels lifted)
| Enter | Dashboard | Live view |
|---|---|---|
| `map FULR` | `[ACK] MAP ... -> ACCEPTED` | no change |
| `start` | `ACCEPTED` | `vehicle_state: 1` |
| `stop` | `ACCEPTED` | `vehicle_state: 1` |
| `replay` | `STALE_SEQ` | `vehicle_state: 1` |
| `stale` | `BAD_SESSION` | `vehicle_state: 1` |

### Line following and E-STOP (wheels lifted, fingers clear)
| Do or enter | Look for |
|---|---|
| Tape under centre, `start` | `vehicle_state: 2`, both speeds about 100 |
| Tape under left only | `line: 2`, `speed_l` about 0, `speed_r` about 90 |
| Remove tape | state 3 for about 1.5 s, then 1, wheels stop |
| Tape back under centre | wheels restart, state 2 |
| `stop` | state 1, wheels stay stopped |
| `start`, then `e` | `ESTOP` event, `estop: 1`, state 6 |
| `start` again | still state 6 |
| wait 1 s, `reset` | `estop: 0`, state 1 |

### Obstacle
Tape under centre, `start`, hand about 10 cm in front. Look for state 5, servo sweep, `OBSTACLE` event. Wheels may move after the scan. Then `e`, `reset`.

### Barcode
`map` with four different characters including the one on the card (e.g. `map ABCD`). Slide the card under the barcode sensor (GP16) both ways. Look for a `BARCODE` event; `value` is the mapped action, not the letter:

| `value` | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|
| Action | Left | Right | Forward | U-turn | Not mapped |

### Wi-Fi recovery and stability
- Ctrl+C the broker, wait 10 s (note whether wheels stay stopped), restart it. Expect `ONLINE` again and `reconn=1`, then `stop` gets `ACCEPTED`.
- Leave online 5 min: `reconn` unchanged and S2's `deadline_misses` unchanged (S1-M06).

### What needs a track
Real line following, barcodes at speed, junction moves (graded), hump height, driving around an obstacle. A black-tape cross on the floor covers most of it.

---

## 3. Vehicle states

| # | Name | Meaning | Wheels |
|---|---|---|---|
| 0 | INIT | S2 not ready yet | stopped |
| 1 | IDLE | ready, not driving | stopped |
| 2 | LINE_FOLLOW | driving, line seen | driving |
| 3 | LINE_SEARCH | driving, line lost; gives up after 1.5 s and goes to 1 | steering |
| 4 | BARCODE_NAV | junction move (advance, then turn) | moving |
| 5 | OBSTACLE_AVOID | S5 stopping, scanning, going around | stopped, then moving |
| 6 | SAFE_STOP | E-STOP, impact, motor fault or ultrasonic fault; `start` refused | stopped |

Priority if several apply: 0, 6, 5, 4, then 2/3, else 1 (`subsystem1/team_adapters.c:176-193`).

`faults` bits: 4 = motors/encoders, 8 = line/barcode, 16 = tilt sensor, 32 = ultrasonic (they add up).
To leave state 6: fix the cause, keep the wheels still, send `reset`, then `start`.

---

## 4. Hardware problems found

### GP1 ECHO without a divider
HC-SR04 ECHO is 5 V; Pico pins take about 3.3 V. Fix:
```
ECHO ---[ 1 kΩ ]---+--- GP1
                   |
                [ 2 kΩ ]
                   |
                  GND
```
Shared GND between sensor, resistor and Pico. Sensor VCC about 5 V. TRIG straight to GP0.

If it still faults: measure ECHO with a meter (exactly 0 V = sensor not responding; flickering at both ECHO and GP1 but still faulting = GP1 may be damaged, and ECHO can be moved to a spare pin in firmware). Try another HC-SR04 if available.

### Left encoder (GP6)
The code path is fine (count updates even in state 6), so GP6 is seeing no pulses.

The motor connector is PH2.0-6P. Wiring per motor, using phase A only (firmware reads one channel):

| Pin | Function | Left | Right |
|---|---|---|---|
| 1 | Encoder GND | Pico GND | Pico GND |
| 2 | Output A | GP6 | GP7 |
| 3 | Output B | not connected | not connected |
| 4 | Encoder power | 3V3(OUT) | 3V3(OUT) |
| 5, 6 | Motor +/- | motor driver | motor driver |

- Never connect pins 5/6 to the Pico.
- Check the encoder supports 3.3 V; if it needs 5 V, output A needs a divider.
- Output A stuck near 0 V while turning: add a 10 kΩ pull-up from A to 3V3 (firmware enables no pull-up).
- Swap test: swap the GP6/GP7 signal wires. If the left wheel then counts on `enc_r`, suspect the GP6 wire/pin; if not, the left encoder or its wiring.
- Then measure ticks per wheel turn (note `enc_l`, turn exactly once, subtract, average a few) and update `B2_CAL_TICKS_PER_WHEEL_REV`.
- Don't drive until fixed: driving with no encoder pulses gives a motor fault (`faults: 4`).

### Left line sensor (GP2)
- Code reads left and centre separately; nothing blocks `line: 2`.
- On these modules the output LED is usually on over white, off over black, and stays steady rather than blinking. Compare with the centre sensor.
- Check power LED, adjustment screw, that the IR emitter/receiver aren't bent, and measure output (0 V / 3.3 V). Over 3.3 V means the module is on 5 V, which is too high for the Pico (check GP4 and GP16 too).
- Swap test: swap GP2 and GP4 signal wires to tell sensor fault from pin fault.

### Tilt
- State 1 to 6 during the tilt test was the ultrasonic fault returning about 8.2 s after `reset` (no IMPACT, `estop: 0`), not the tilt.
- A sharp movement can trigger an impact stop (`estop: 1`), after which `pitch` freezes until `reset`.
- Pitch uses only the sensor's X axis (`subsystem4/imu.c:170`). Earlier sessions held 12 to 33, so the sensor works.
- Test: restart flat, then hold each for 3 s: nose up, nose down, left side up, right side up.
  - Nose up about +20: fine (S1-M03 pass).
  - Nose up about -20: flip `forward_axis_sign` or turn the module.
  - Only side tilts change it: module mounted sideways; rotate 90° or change the axis in firmware.
  - Nothing holds: module loose, or frozen after an impact.

---

## Next steps
1. Add the ECHO divider and get `obs_state: 0`, `faults: 0`.
2. Fix the left encoder wiring; measure ticks per wheel turn and update the setting.
3. Sort out the left line sensor.
4. Run the four-position tilt test and fix the axis if needed.
5. Re-run the bench procedure, record results in `testing/subsystem1/TEST_LOG.md`, then test on a track.
