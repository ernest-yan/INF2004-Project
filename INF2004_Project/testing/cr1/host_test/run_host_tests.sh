#!/bin/sh
# CR1 host tests. Run from anywhere inside the repo:
#   sh INF2004_Project/testing/cr1/host_test/run_host_tests.sh
# Needs gcc and git (the S5 test rebuilds the pre-CR1 profile.c from the
# git tag pre-cr1 to compare old and new results).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
PROJ=$(cd "$HERE/../../.." && pwd)
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
CC="gcc -std=gnu11 -Wall -Wextra -O2"
fail=0

echo "== S1 telemetry codec (wire format; only BARCODE events gained the letter) =="
$CC -I"$PROJ/app_program/subsystem1" "$PROJ/app_program/subsystem1/tlm_codec.c" \
    "$PROJ/testing/subsystem1/host_test/test_tlm_codec.c" -o "$OUT/codec" && "$OUT/codec" | tail -1 || fail=1

echo "== S1 adapters + system fault mask =="
$CC "$HERE/test_s1_adapters.c" -o "$OUT/s1" && "$OUT/s1" || fail=1

echo "== S2 motion =="
$CC "$HERE/test_s2_motion.c" "$PROJ/app_program/subsystem2/b2_math.c" -o "$OUT/s2" && "$OUT/s2" || fail=1

echo "== S5 obstacle profile: pre-cr1 vs CR1 =="
mkdir -p "$OUT/old"
for f in profile.c profile.h; do
    git -C "$PROJ" show "pre-cr1:INF2004_Project/app_program/subsystem5/$f" > "$OUT/old/$f"
done
$CC -I"$OUT/old" -I"$HERE" -DWRAP_NAME=old_analyze \
    -Dsubsystem5_profile_analyze=old_profile_analyze -Dsubsystem5_cosd=old_cosd \
    -c "$HERE/s5_wrap.c" -o "$OUT/old.o"
$CC -I"$PROJ/app_program/subsystem5" -I"$HERE" -DWRAP_NAME=new_analyze \
    -Dsubsystem5_profile_analyze=new_profile_analyze -Dsubsystem5_cosd=new_cosd \
    -c "$HERE/s5_wrap.c" -o "$OUT/new.o"
$CC -I"$HERE" "$HERE/test_s5_profile.c" "$OUT/old.o" "$OUT/new.o" -o "$OUT/s5" && "$OUT/s5" || fail=1

echo "== S3 barcode decoder + map: pre-cr1 vs CR1 =="
mkdir -p "$OUT/oldtree"
git -C "$(git -C "$PROJ" rev-parse --show-toplevel)" archive --prefix=INF2004_Project/app_program/ pre-cr1:INF2004_Project/app_program | tar -x -C "$OUT/oldtree"
S3CC="gcc -std=gnu11 -O2 -w"
$S3CC -I"$OUT/oldtree/INF2004_Project/app_program/subsystem3" "$HERE/s3_driver.c" -o "$OUT/s3_old"
$S3CC -I"$PROJ/app_program/subsystem3" "$HERE/s3_driver.c" -o "$OUT/s3_new"
"$OUT/s3_old" > "$OUT/s3_old.txt" 2>/dev/null
"$OUT/s3_new" > "$OUT/s3_new.txt" 2> "$OUT/s3_new_err.txt"
scans=$(grep -c '^scan' "$OUT/s3_new.txt")
decoded=$(grep '^scan' "$OUT/s3_new.txt" | grep -vc -- '-> -$' || true)
right=$(grep '^scan' "$OUT/s3_new.txt" | awk '{if ($3 == $NF) n++} END {print n+0}')
wrong=$((decoded - right))
old_right=$(grep '^scan' "$OUT/s3_old.txt" | awk '{if ($3 == $NF) n++} END {print n+0}')
# 2026-11-19: the decoder now corrects bar widening (IR spot), so its letters
# no longer have to match pre-cr1 scan for scan.  It must never decode a
# WRONG letter; the movement maps must still be identical.
grep -v '^scan' "$OUT/s3_old.txt" > "$OUT/s3_old_maps.txt"
grep -v '^scan' "$OUT/s3_new.txt" > "$OUT/s3_new_maps.txt"
if [ "$wrong" -ne 0 ]; then
    echo "S3: new decoder gave $wrong WRONG letters:"; grep '^scan' "$OUT/s3_new.txt" | awk '$NF != "-" && $3 != $NF' | head -20; fail=1
elif diff "$OUT/s3_old_maps.txt" "$OUT/s3_new_maps.txt" > "$OUT/s3_diff.txt"; then
    echo "S3: $scans simulated scans: $right correct letter, 0 wrong (pre-cr1: $old_right correct); maps identical"
    undecoded=$(grep '^scan' "$OUT/s3_new.txt" | grep -c -- '-> -$' || true)
    seen=$(sed -n 's/failed-scan events: //p' "$OUT/s3_new_err.txt")
    if [ "$seen" = "$undecoded" ]; then
        echo "S3: every undecoded scan reported as 'seen, not decoded' ($seen of $undecoded)"
        diag=$(sed -n 's/scan diagnosis wrong: //p' "$OUT/s3_new_err.txt")
        case "$diag" in
            "0 "*) echo "S3: every scan reports its result and element count ${diag#0 }" ;;
            *) echo "S3: scan diagnosis wrong: $diag"; fail=1 ;;
        esac
    else
        echo "S3: 'seen, not decoded' count $seen != undecoded scans $undecoded"; fail=1
    fi
else
    echo "S3: barcode -> movement maps DIFFER:"; head -20 "$OUT/s3_diff.txt"; fail=1
fi

echo "== S3 junction turns + decoding while line following (s3_junction_sim.c) =="
# The real line_barcode.c drives a simulated car over a cross junction.
# Known result (26 Nov): 40 of 41 junction runs; the one failure is the
# "left stub between barcode and junction" course (a sharp left curve there
# is taken as the junction), see app_program/subsystem3/LINE_BARCODE.md §6.
# So fewer than 40, or any wrongly decoded letter, fails the suite.
$S3CC -I"$PROJ/app_program/subsystem3" "$HERE/s3_junction_sim.c" -o "$OUT/s3sim" -lm
"$OUT/s3sim" > "$OUT/s3sim.txt" || true
jok=$(sed -n 's#^\([0-9]*\)/\([0-9]*\) junction runs correct#\1#p' "$OUT/s3sim.txt")
jall=$(sed -n 's#^\([0-9]*\)/\([0-9]*\) junction runs correct#\2#p' "$OUT/s3sim.txt")
dsum=$(awk '/correct, .* wrong, .* not decoded/ {
    for (i = 1; i <= NF; i++) {
        if ($(i+1) == "correct,") { split($i, a, "/"); ok += a[1]; all += a[2] }
        if ($(i+1) == "wrong,") { bad += $i }
    }
} END { printf "%d %d %d", ok, all, bad }' "$OUT/s3sim.txt")
set -- $dsum
if [ -n "$jok" ] && [ "$jok" -ge 40 ] && [ "$3" -eq 0 ]; then
    echo "S3: junction turns $jok/$jall correct; barcodes while line following $1/$2 decoded, $3 wrong"
    grep 'FAIL' "$OUT/s3sim.txt" | sed 's/^ */S3: known limitation (left stub before the junction): /' || true
else
    echo "S3: junction simulator REGRESSED (junction ${jok:-?}/${jall:-?}, wrong letters $3):"
    grep -E 'FAIL|wrong' "$OUT/s3sim.txt" | head -10; fail=1
fi

echo "== S4 IMU (lift / overturn / hump): pre-cr1 vs CR1 =="
$S3CC -I"$OUT/oldtree/INF2004_Project/app_program/subsystem4" "$HERE/s4_driver.c" -o "$OUT/s4_old"
$S3CC -I"$PROJ/app_program/subsystem4" "$HERE/s4_driver.c" -o "$OUT/s4_new"
"$OUT/s4_old" > "$OUT/s4_old.txt"
"$OUT/s4_new" > "$OUT/s4_new.txt"
lines=$(wc -l < "$OUT/s4_new.txt")
lifts=$(grep -c 'EVENT type=1' "$OUT/s4_new.txt" || true)
humps=$(grep -c 'EVENT type=0' "$OUT/s4_new.txt" || true)
if diff "$OUT/s4_old.txt" "$OUT/s4_new.txt" > "$OUT/s4_diff.txt" &&
   grep -q 'lock errors (nested or unbalanced): 0 / depth 0' "$OUT/s4_new.txt"; then
    echo "S4: 12 simulated runs, $lines status/event lines IDENTICAL ($humps humps, $lifts lift/overturn stops); locks balanced"
else
    echo "S4: old and new DIFFER or lock error:"; head -20 "$OUT/s4_diff.txt"; tail -1 "$OUT/s4_new.txt"; fail=1
fi

echo "== BARCODE event: codec -> dashboard.py =="
$CC -I"$PROJ/app_program/subsystem1" "$PROJ/app_program/subsystem1/tlm_codec.c" \
    "$HERE/barcode_event_enc.c" -o "$OUT/bce" && "$OUT/bce" > "$OUT/frames.txt"
python3 -I "$HERE/test_barcode_event.py" "$PROJ/testing/subsystem1/tools/dashboard.py" < "$OUT/frames.txt" || fail=1

echo "== MAP command: dashboard.py <-> C codec =="
$CC -I"$PROJ/app_program/subsystem1" "$PROJ/app_program/subsystem1/tlm_codec.c" \
    "$HERE/map_cmd_enc.c" -o "$OUT/mce"
python3 -I "$HERE/test_map_cmd.py" "$PROJ/testing/subsystem1/tools/dashboard.py" "$OUT/mce" || fail=1

[ $fail -eq 0 ] && echo "ALL CR1 HOST TESTS PASSED" || { echo "CR1 HOST TESTS FAILED"; exit 1; }
