# BARR-C:2018 compliance

The project must conform to the Barr Group *Embedded C Coding Standard* (BARR-C:2018) (Project Write-up, p. 1; design review NFR4). This page records what the application code follows, the known deviations, and how they were checked.

**Scope:** the team's application code under `app_program/` (subsystem1 to subsystem5 and `integration/`): 44 files, about 10,400 lines. The micro T-Kernel sources, the Pico SDK and the template's `device/`, `kernel/`, `lib/` folders are third-party and out of scope.

**Fixes applied (23 Nov 2026):** braces on the 6 unbraced `if` bodies (S3), 4 of 5 `continue` statements removed (S2, S3), the 4 missing file header comments in S2 and S4 added, and every tab in S1 to S4 converted to spaces (4-space indentation). Checked with `git diff -w` (only the intended code changed), the host test suite (results identical before and after) and both firmware builds (`make NET=lwip MQTT=1`, `make TEST=1 NET=lwip MQTT=1`, zero warnings). Not yet re-flashed on the car.

**Method (23 Nov 2026):** a mechanical scan for the rules a script can check (commands in §4), plus compiler warnings (`-Wall -Wextra`, zero warnings in the demo and test builds, see [CR1_REPORT.md](testing/cr1/CR1_REPORT.md) §3). No commercial MISRA / BARR-C checker was available, so rules that need a full parser were reviewed by hand, not exhaustively.

## 1. Rules followed

| Area | Rule | How the code meets it |
|---|---|---|
| Data types | Fixed-width integers from `<stdint.h>` | 1,069 uses of `uint8_t`…`int64_t`; plain `int` is limited to the cases in §2 |
| Data types | No signed bit-fields | All bit-fields are unsigned; signed values are stored offset-binary (e.g. S2 wheel speeds, `motion.c`) |
| Data types | `U` suffix on unsigned constants | Used throughout configuration and limits (`930U`, `650U`, `0x19U`) |
| Keywords | No `goto`, `auto` or `register` | 0 occurrences |
| Headers | Every header has an include guard | 44 / 44 files checked: none missing |
| Naming | Module prefix on public names | `s1_` / `tlm_` (S1), `b2_` (S2), `s3_` (S3), `b4_` (S4), `subsystem5_` (S5). S2's `math.*` was renamed `b2_math.*` (19 Nov) so it cannot clash with the C library header |
| Naming | `s_` for file-scope statics, `p_` for pointers in the newer modules | `s_lock`, `s_status`, `p_ticks` |
| Scope | File-scope data and helpers are `static` | All module state is `static`; other modules reach it only through the public API (IF-01 to IF-18 in [TEAM.md](TEAM.md)) |
| Shared data | Shared data accessed under a lock or by a single writer | Reviewed for CR1, every case listed in [CR1_REPORT.md](testing/cr1/CR1_REPORT.md) §3 "Concurrency" |
| Comments | `/* */` comments; a header comment at the top of each file | No `//` comments in the application code. 42 of 44 files start with a header comment (the 2 missing are in S5, §2) |
| Layout | Spaces, not tabs; 4-space indentation | All of S1 to S4 (tabs remain only in S5 and `integration/`, §2) |
| Statements | Braces around every `if` / `while` / `for` body | 0 unbraced bodies |
| Constants | `#define` constants in UPPER_CASE, parenthesised | e.g. `B2_INTEGRAL_LIMIT (8000)`, `B4_HUMP_START_DEG (2.0f)` |

## 2. Known deviations

| Rule | Where (count) | Reason / plan |
|---|---|---|
| **No tab characters** | 5 files: S5 (`ultrasonic.c`, `ultrasonic.h`, `test.c`) and `integration/vehicle_ctrl.c` / `.h` | Fixed in S1 to S4 on 23 Nov. S5 belongs to its owner and `integration/` to the team; the same whitespace-only conversion applies |
| **Left brace on its own line** | 234 places, 101 of them in `line_barcode.c` (S3, K&R style); `team_adapters.c` 25, `task_telemetry.c` 22, `drv_i2c1.c` 18 | Style only. Some hits are braced one-line guards such as `if (x) { return y; }` in S4, which keep the braces the standard requires. The 23 Nov S3 fixes follow the file's existing K&R style for consistency |
| **No `continue`** | 1: `line_barcode.c`, the line-following task loop (after starting a junction turn) | The other 4 were removed on 23 Nov. This one sits in the main state machine, which has no host test and cannot be re-tested on the car before submission, so it was left as is |
| **File header comment** | S5: `ultrasonic.h`, `profile.h` | Documentation only |
| **Lines ≤ 80 characters** | 179 lines, mostly `ultrasonic.c` (59) and `ultrasonic.h` (19) (S5), then `line_barcode.h` (12), `team_adapters.c` (11) | Long table rows and comments; no logic impact. (The count rose from 144 because a tab used to count as one character; the lines did not get longer on screen) |
| **Fixed-width types instead of `int` / `char`** | `drv_i2c1.c` (`int` return codes), test files' loop counters, `for (int i …)` in `line_barcode.c`, `const int` results in `imu.c` | The Pico SDK I2C functions return `int`, so those are kept to match the SDK. `char` is used only for text (barcode letters, JSON buffers), which the standard permits |

## 3. Summary

The safety-relevant rules (fixed-width types, unsigned bit-fields, no `goto`, include guards, `static` module state, locked shared data) are followed across all subsystems. After the 23 Nov fixes, the remaining deviations are layout (brace position, line length), tabs in S5 and `integration/`, two S5 header comments, and one `continue` in S3's line-following loop. That `continue` is the only remaining deviation that touches control flow.

## 4. How to reproduce the scan

From `INF2004_Project/app_program`:

```sh
F=$(find subsystem1 subsystem2 subsystem3 subsystem4 subsystem5 integration -name "*.[ch]")
grep -P -c '\bgoto\b' $F | grep -v ':0$'                                  # goto
grep -n -P '\bcontinue;' $F                                               # continue
grep -c -P '\t' $F | grep -v ':0$'                                        # tabs
for f in $F; do awk -v f=$f 'length>80{n++} END{if(n) print n, f}' $f; done # long lines
for h in $(echo $F | tr ' ' '\n' | grep '\.h$'); do grep -q '#ifndef' $h || echo "no guard: $h"; done
grep -c -P '^\s*(if|for|while|switch|else|}\s*else)\b.*\{\s*(/\*.*)?$' $F | grep -v ':0$'   # brace on same line
grep -c -P '^\s*(if|while|for)\s*\(.*\)\s*[^{\s].*;\s*$' $F | grep -v ':0$'               # body without braces
```
