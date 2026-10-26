# Ernest's Line Following + Barcode Decoding Starter

Files:
- `line_barcode.h` — API and data structures
- `line_barcode.c` — line classification and timestamped barcode edge capture
- `pico_integration_example.c` — Pico SDK integration template
- `CMakeLists.txt` — small Pico SDK build template

## Based on the team resources
The project write-up assigns Ernest: IR integration/calibration, line-position
estimation, line-following, junction detection, barcode detection/decoding, and
navigation command generation. The barcode/line drawing specifies an 18 mm
line, an approximately 20 mm gap/offset, and approximately 141 mm barcode
length. The document does **not** define the actual A/B/C/D bar patterns.

## Proposed pin mapping (must verify against physical wiring)
- GP26 / ADC0: left analogue IR sensor
- GP27 / ADC1: centre analogue IR sensor
- GP16: offset digital barcode sensor

These are the design draft's proposed GPIO assignments, not proof that the car
is wired this way. Verify sensor labels, logic levels and actual polarity first.

## Important limitations before using on the car
1. The default ADC thresholds are placeholders. Log readings over white floor,
   black line and barcode marks, then choose thresholds with margin.
2. `lb_decode_barcode()` intentionally returns `LB_CMD_NONE` until the team
   agrees and measures the four barcode patterns. Do not invent the mapping.
3. The two-sensor line estimate is deliberately simple; tune the sign, sensor
   geometry, junction logic and control gain on the actual robot.
4. This code does not drive motors. Send line error and navigation events to
   `MotionControl` / `VehicleController`; the report says `MotionControl` is
   the sole owner of motor outputs.
5. The example loop uses Pico SDK timing, not a micro T-Kernel task. Move the
   calls into the team's `LineBarcode` task and use the required RTOS timing/
   queue API before integrating with the final firmware.
6. A rising/falling GPIO interrupt may run in interrupt context. Keep its work
   short. If using RTOS APIs, use only the RTOS's documented ISR-safe variants.

## Build
Copy the files into the team's Pico SDK project and add `line_barcode.c` to
the target sources. The CMake example assumes the normal Pico SDK import setup.

## Updated GP16 reader behaviour (autonomous demo mode)
- GP2 and GP4 line tracking still runs on every call and is not gated by barcode capture.
- GP16 capture is enabled by default; no serial `s` command or USB connection is required. It continuously looks for candidate stripe patterns and resets after the 50 ms quiet gap so it can detect a later barcode.
- This assumes GP16 is physically offset from the normal track line and only crosses the barcode lane. If GP16 can see ordinary track marks, use a reliable zone/navigation trigger or add a physical/optical discriminator; software alone cannot distinguish identical black/white patterns.
- All edge durations use `time_us_32()` microseconds; the legacy encoder-tick argument is ignored.

## Encoding limitation
The supplied PDF shows stripe patterns and labels `A` on two pages, but does not specify a barcode standard or a complete pattern-to-command table. The existing project had a small proposed Code-39-style table; this implementation retains it as a provisional mapping and rejects ambiguous/incomplete scans. It is **not** a universal barcode decoder and must not be treated as validated until the instructor's symbology/pattern mapping is confirmed and tested against measured sensor traces. The sensor also cannot distinguish an ordinary line from a barcode if GP16 physically crosses the same black/white pattern; scan gating prevents accidental decode outside the expected scan window.
