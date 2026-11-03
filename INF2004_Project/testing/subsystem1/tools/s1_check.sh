#!/usr/bin/env bash
#
# s1_check.sh -- Buddy 1 "am I still merge-safe?" check.
#
# Run it before every commit, from anywhere inside the repo:
#     bash testing/subsystem1/tools/s1_check.sh
#
# It compares your branch (committed + uncommitted + new files) with the
# base you started from (team/main if you have added the team remote,
# otherwise origin/main) and sorts every changed file into:
#   OK       Buddy 1's own paths -- these are what you will merge
#   SANDBOX  side effects of tools/clean_slate.sh (subsystems 2-5);
#            the merge script hands these back to the team's version
#   WARNING  anything else -- the merge script will DISCARD these, so
#            move the change into your folder or agree it with the team
# and makes sure config/wifi_secrets.h is not tracked by git.
#
# Exit code: 0 = safe, 1 = warnings or a tracked secret, 2 = setup problem.

set -u

top=$(git rev-parse --show-toplevel 2>/dev/null) || { echo "Not inside a git repo."; exit 2; }
cd "$top" || exit 2

base_ref="${1:-}"
if [ -z "$base_ref" ]; then
	if git rev-parse -q --verify team/main >/dev/null; then
		base_ref=team/main
	elif git rev-parse -q --verify origin/main >/dev/null; then
		base_ref=origin/main
	else
		echo "No team/main or origin/main to compare with. Run 'git fetch' first."
		exit 2
	fi
fi
base=$(git merge-base HEAD "$base_ref") || { echo "No shared history with $base_ref."; exit 2; }

mine='^(app_program/subsystem1/|testing/subsystem1/|lib/liblwip/sysdepend/pico_rp2040/mqtt_utk\.c$)'
sandbox='^(tools/clean_slate\.sh$|app_program/subsystem[2-5]/)'

changed=$( { git diff --name-only "$base"; git ls-files --others --exclude-standard; } | sort -u)

n_ok=0; n_sb=0; n_warn=0
warn_list=""
while IFS= read -r f; do
	[ -z "$f" ] && continue
	if [[ "$f" =~ $mine ]]; then
		n_ok=$((n_ok + 1))
	elif [[ "$f" =~ $sandbox ]]; then
		n_sb=$((n_sb + 1))
	else
		n_warn=$((n_warn + 1))
		warn_list="$warn_list  $f"$'\n'
	fi
done <<< "$changed"

secret=0
if git ls-files --error-unmatch config/wifi_secrets.h >/dev/null 2>&1; then
	secret=1
fi

echo "Compared with: $base_ref (base ${base:0:7})"
echo "  OK       $n_ok file(s) in Buddy 1's paths"
echo "  SANDBOX  $n_sb file(s) from clean_slate.sh (team's version wins at merge)"
if [ "$n_warn" -gt 0 ]; then
	echo "  WARNING  $n_warn file(s) outside your paths (would be DISCARDED at merge):"
	printf "%s" "$warn_list"
fi
if [ "$secret" -eq 1 ]; then
	echo "  ERROR    config/wifi_secrets.h is tracked by git! Run:"
	echo "           git rm --cached config/wifi_secrets.h && git commit -m 'Untrack secrets'"
fi

if [ "$n_warn" -eq 0 ] && [ "$secret" -eq 0 ]; then
	echo "Merge-safe."
	exit 0
fi
exit 1
