#!/usr/bin/env bash
#
# s1_merge_team.sh -- merge the team repo into your Buddy 1 sandbox branch.
#
# Usage (from anywhere inside your sandbox, on your buddy1 branch):
#     bash testing/subsystem1/tools/s1_merge_team.sh [team-repo-url] [--no-build]
# Default URL: https://github.com/ernest-yan/INF2004-Project.git
#
# The rule it applies:  Buddy 1's paths   -> YOUR version wins
#                       everything else    -> the TEAM's version wins
# Buddy 1's paths = app_program/subsystem1/, testing/subsystem1/,
#                   lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c
#
# So the result is exactly "team/main + your files". Your commits (and the
# commit hashes in your TEST_LOG) are kept. Nothing is pushed.
#
# Safety: it stops before changing anything if your work isn't committed,
# if the team repo isn't on the template yet, or if the team repo still has
# the template examples. It tags your branch first, so you can always undo:
#     git merge --abort                 (if it stopped half-way)
#     git reset --hard <the tag it prints>   (after it finished)

set -u

url="https://github.com/ernest-yan/INF2004-Project.git"
do_build=1
for a in "$@"; do
	case "$a" in
	--no-build) do_build=0 ;;
	*) url="$a" ;;
	esac
done

mine='^(app_program/subsystem1/|testing/subsystem1/|lib/liblwip/sysdepend/pico_rp2040/mqtt_utk\.c$)'
utk='lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c'

say()  { printf '%s\n' "$*"; }
stop() { printf 'STOP: %s\n' "$*"; exit 1; }

top=$(git rev-parse --show-toplevel 2>/dev/null) || stop "not inside a git repo."
cd "$top" || exit 1
branch=$(git symbolic-ref --short -q HEAD) || stop "not on a branch (detached HEAD)."

# ---- 1. preconditions (nothing changed yet) ---------------------------------
git rev-parse -q --verify MERGE_HEAD >/dev/null && stop "a merge is already in progress (git merge --abort)."
[ -z "$(git status --porcelain --untracked-files=no)" ] || stop "commit your changes first (git status)."
if git ls-files --error-unmatch config/wifi_secrets.h >/dev/null 2>&1; then
	stop "config/wifi_secrets.h is tracked by git. Untrack it first: git rm --cached config/wifi_secrets.h"
fi

# ---- 2. fetch the team repo -------------------------------------------------
if git remote get-url team >/dev/null 2>&1; then
	git remote set-url team "$url"
else
	git remote add team "$url"
fi
say "Fetching $url ..."
git fetch -q team || stop "could not fetch the team repo."
git rev-parse -q --verify team/main >/dev/null || stop "team repo has no 'main' branch."

git cat-file -e team/main:app_program/app_main.c 2>/dev/null \
	|| stop "the team repo isn't on the project template yet. Wait for the migration (IS33_Team_Repo_Migration.md, Phase 1)."
base=$(git merge-base HEAD team/main) \
	|| stop "no shared history with the team repo (made with 'Use this template' or a ZIP?). Use the copy method instead (migration doc, Phase 2 B)."
if git grep -q -E 'tid_led|S1_PRI_LED' team/main -- 'app_program/subsystem[2-5]/*' 2>/dev/null; then
	stop "the team repo still has the template examples (subsystem 4 controls subsystem 1's LED task). Ask for tools/clean_slate.sh to be run there first (migration doc, Phase 1 step 5)."
fi
if ! git diff --quiet "$base" team/main -- "$utk"; then
	say "NOTE: the team repo has also changed $utk. Your version will be kept;"
	say "      compare afterwards with: git diff team/main -- $utk"
fi

if git merge-base --is-ancestor team/main HEAD; then
	say "Nothing to merge: your branch already contains everything in team/main."
	exit 0
fi

# ---- 3. safety tag, then merge ----------------------------------------------
tag="before-team-merge-$(date +%Y%m%d-%H%M%S)"
n=1
while git rev-parse -q --verify "refs/tags/$tag" >/dev/null; do
	tag="before-team-merge-$(date +%Y%m%d-%H%M%S)-$n"; n=$((n + 1))
done
git tag "$tag" || stop "could not create the safety tag."
say "Safety tag: $tag   (undo everything later with: git reset --hard $tag)"

git merge --no-ff --no-commit team/main >/dev/null 2>&1
git rev-parse -q --verify MERGE_HEAD >/dev/null \
	|| stop "git merge did not start. Your branch is unchanged (safety tag $tag)."

# ---- 4. resolve conflicts by the rule ---------------------------------------
while IFS= read -r p; do
	[ -z "$p" ] && continue
	if [[ "$p" =~ $mine ]]; then side=2; name=ours; who="yours"; else side=3; name=theirs; who="team's"; fi
	if git ls-files -u -- "$p" | awk '{print $3}' | grep -qx "$side"; then
		git checkout --"$name" -- "$p" && git add -- "$p"
		say "  conflict  $p  -> kept $who"
	else
		git rm -q -f -- "$p" >/dev/null
		say "  conflict  $p  -> deleted ($who side doesn't have it)"
	fi
done < <(git diff --name-only --diff-filter=U)

# ---- 5. outside your paths, make everything identical to the team -----------
while IFS= read -r p; do
	[ -z "$p" ] && continue
	[[ "$p" =~ $mine ]] && continue
	if git cat-file -e "team/main:$p" 2>/dev/null; then
		git checkout team/main -- "$p"
		say "  restored  $p  (team's version)"
	else
		git rm -q -f -- "$p" >/dev/null
		say "  removed   $p  (not in the team repo)"
	fi
done < <(git diff --cached --name-only team/main)

# ---- 6. check and commit ----------------------------------------------------
left=$(git diff --cached --name-only team/main | grep -Ev "$mine" || true)
[ -z "$left" ] || stop "unexpected differences remain outside your paths:
$left
Nothing is committed. Undo with: git merge --abort"

git commit -q -m "Merge team/main into $branch (Buddy 1 keeps its paths, team keeps the rest)" \
	|| stop "commit failed. Undo with: git merge --abort"
say ""
say "Merged. Your branch = team/main + these Buddy 1 files:"
git diff --stat team/main HEAD | tail -n 1

# ---- 7. build check ---------------------------------------------------------
if [ "$do_build" -eq 1 ]; then
	if [ -z "${PICO_SDK_PATH:-}" ] || [ ! -f config/wifi_secrets.h ]; then
		say "Build check skipped (needs PICO_SDK_PATH and config/wifi_secrets.h)."
	else
		say "Build check: make NET=lwip MQTT=1 ..."
		if (cd build_make && make NET=lwip MQTT=1 -j8 >/tmp/s1_merge_build.log 2>&1); then
			say "Build OK."
		else
			say "BUILD FAILED -- see /tmp/s1_merge_build.log. Nothing was pushed."
			say "Undo the merge with: git reset --hard $tag"
			exit 1
		fi
	fi
fi

say ""
say "Next:"
say "  1. Flash and re-run your quick tests (guide Stage 9.2)."
say "  2. git push -u team $branch      then open a pull request into main on GitHub."
