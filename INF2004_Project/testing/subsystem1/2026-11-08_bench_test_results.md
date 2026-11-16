# Bench test results: 8 November 2026

## In one sentence

The car can now **follow a black line on its own, steer when the line moves, stop by itself
when it loses the line, and be stopped safely from the laptop at any time**. All 9 tests passed.

## What we tested, and how

We tested the car **"on the bench"**: it sits on a stand with its wheels off the table, so the
wheels can spin freely without the car driving away. A strip of black electrical tape held under
the car stands in for the track's line.

The laptop talks to the car over Wi-Fi. We type commands such as `start`, `stop` and `e`
(emergency stop) into the laptop, and the car sends back live readings five times a second,
which we read on screen. The car also stores everything in a log file, so every result
below can be checked afterwards.

Terms used below:

| Term | Meaning |
|---|---|
| **Line sensor** | Small sensors under the car that see whether the black tape is under them (left or centre) |
| **Encoder** | A counter inside each motor that ticks as the wheel turns. It tells the car how fast each wheel is really going |
| **Emergency stop (E-STOP)** | A stop that locks the car until someone deliberately unlocks it with `reset` |
| **Ultrasonic sensor** | The "eyes" at the front that measure distance to obstacles, like a bat |

## Results

| # | What we checked | Why it matters | What happened | Result |
|---|---|---|---|---|
| 0 | The car starts up with no warnings | Every test starts from a clean, known state | No faults, car ready and stopped | ✅ Pass |
| 1 | With the line under it, `start` makes it drive | The basic job: drive along the line | Both wheels ran smoothly at the target speed (100 mm per second) for 80 seconds | ✅ Pass |
| 2 | Moving the line to the left makes it turn left | The car has to follow curves in the track | The left wheel stopped and the right wheel kept going, which turns the car left | ✅ Pass |
| 3 | Taking the line away makes it stop by itself | If the car loses the track, it must not drive off the table | It searched for about 1.5 seconds, then stopped on its own (4 out of 4 times) | ✅ Pass |
| 4 | Putting the line back makes it carry on, with no new `start` | Small gaps in the tape shouldn't need a person to restart it | It started again by itself every time (4 out of 4) | ✅ Pass |
| 5 | `stop` stops it, and it stays stopped | A normal stop must not undo itself | It stopped and stayed still for 6–10 seconds, even with the line still under it | ✅ Pass |
| 6 | `e` (emergency stop) stops it immediately | The most important safety feature | It stopped at once and reported "emergency stop, by the operator" | ✅ Pass |
| 7 | After an emergency stop, `start` does nothing | Nobody can restart it by accident during an emergency | The car received the `start` but stayed locked for 12 seconds | ✅ Pass |
| 8 | `reset` unlocks the car again | We can recover without switching it off | It unlocked with no warnings, and the next `start` drove normally | ✅ Pass |

## Problems found and fixed today

Before these tests passed, four problems had to be found and fixed:

1. **The two motors were plugged into each other's sockets.** Each wheel's speed control was
   reading the *other* wheel's counter, so one wheel raced at full power while the other stopped.
   *Fix:* swapped the motor wires on the board (left motor to M1, right motor to M2).
2. **The ultrasonic sensor's wires had come loose** while the board was being handled, so it
   sometimes stopped answering and the car stopped as a safety measure.
   *Fix:* reseated the wires and cleared the space in front of the sensor.
3. **The car had the wrong idea of how many ticks one wheel turn makes** (20 instead of about
   930), so it thought it was going 46 times faster than it really was and crawled along.
   *Fix:* turned each wheel 10 times by hand, counted the ticks, and set the real value (930).
4. **A software bug made the car think a wheel had jammed when it hadn't.** Once the wheels
   reported their speed properly, the bug stopped the car every few seconds to a minute.
   *Fix:* corrected the timing check in the motor code (`subsystem2/motion.c`).

## Still to test

- Driving around an obstacle (ultrasonic sensor and servo)
- Reading barcode cards (the car's turn instructions)
- Recovering when the Wi-Fi drops, and staying connected for 5 minutes
- Everything above again **on the floor on a real tape track**, not just on the stand

## Evidence

- **Log file:** the dashboard log `2026-11-07_dashboard.csv`, session started 08 Nov 18:10:47,
  tests run 18:11 to 18:20.
- **Firmware:** commit `0a82622` plus today's two changes in `subsystem2/` (ticks per turn
  930, and the false-stall fix).
- **Setup:** Cytron Robo Pico with Pico W, wheels lifted, Mosquitto broker and `dashboard.py`
  on the laptop.
