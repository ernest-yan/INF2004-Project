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

echo "== S2 motion =="
$CC "$HERE/test_s2_motion.c" "$PROJ/app_program/subsystem2/math.c" -o "$OUT/s2" && "$OUT/s2" || fail=1

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

[ $fail -eq 0 ] && echo "ALL CR1 HOST TESTS PASSED" || { echo "CR1 HOST TESTS FAILED"; exit 1; }
