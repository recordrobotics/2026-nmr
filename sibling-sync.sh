#!/usr/bin/env bash

set -euo pipefail

if ! git remote show sibling >/dev/null 2>&1; then
    git remote add sibling https://github.com/recordrobotics/2026-robot.git
fi

git fetch --all
git checkout main
git reset --hard origin/main

origin_hashes=()
origin_subjects=()
origin_last_lines=()
while IFS= read -r -d '' hash &&
      IFS= read -r -d '' subject &&
      IFS= read -r -d '' body; do
    [[ $subject == \[shared\]* ]] || continue
    origin_hashes+=("${hash//[$'\r\n']}")
    origin_subjects+=("$subject")
    last_line=$(printf '%s\n' "$body" | sed '/^[[:space:]]*$/d' | tail -n 1)
    origin_last_lines+=("$last_line")
done < <(
    git log main --format='%H%x00%s%x00%b%x00'
)

sibling_hashes=()
sibling_subjects=()
sibling_last_lines=()
while IFS= read -r -d '' hash &&
      IFS= read -r -d '' subject &&
      IFS= read -r -d '' body; do
    [[ $subject == \[shared\]* ]] || continue
    sibling_hashes+=("${hash//[$'\r\n']}")
    sibling_subjects+=("$subject")
    last_line=$(printf '%s\n' "$body" | sed '/^[[:space:]]*$/d' | tail -n 1)
    sibling_last_lines+=("$last_line")
done < <(
    git log sibling/main --format='%H%x00%s%x00%b%x00'
)

for i in "${!origin_last_lines[@]}"; do
    last_line="${origin_last_lines[$i]}"
    if [[ "$last_line" =~ ^\(cherry\ picked\ from\ commit\ ([0-9a-f]{40})\)$ ]]; then
        picked_from_hash="${BASH_REMATCH[1]}"
        match_index="$i"
        match_commit="${origin_hashes[$i]}"
        match_subject="${origin_subjects[$i]}"
        match_last_line="$last_line"
        match_picked_from="$picked_from_hash"

        match_picked_from_last_body_line=$(git show -s --format='%B' "$match_picked_from" | sed '/^[[:space:]]*$/d' | tail -n 1)
        if [[ "$match_picked_from_last_body_line" =~ ^\(cherry\ picked\ from\ commit\ ([0-9a-f]{40})\)$ ]]; then
            echo "Error: commit $match_commit ($match_subject) was cherry-picked from $match_picked_from, which itself was cherry-picked from ${BASH_REMATCH[1]}. This is officially cooked."
            exit 1
        fi

        if ! git merge-base --is-ancestor "$match_picked_from" sibling/main; then
            echo "Deleting commit $match_commit ($match_subject) from origin/main because it was cherry-picked from $match_picked_from, which is not in the lineage of sibling/main"
            git rebase --onto "$match_commit^" "$match_commit"
            git push --force --force-with-lease --force-if-includes origin main
            exec "$0" "$@"
        fi
    else
        for j in "${!sibling_subjects[@]}"; do
            if [[ "${sibling_subjects[$j]}" == "${origin_subjects[$i]}" ]] &&
                [[ ! "${sibling_last_lines[$j]}" =~ ^\(cherry\ picked\ from\ commit\ ([0-9a-f]{40})\)$ ]]; then
                echo "Error: both origin/main and sibling/main have a commit with the same subject (${origin_subjects[$i]}) and neither have a cherry-pick line. You probably caused this by manually cherry-picking."
                exit 1
            fi
        done
    fi
done

origin_all_subjects=$(git log --remotes=origin --format=%s)
for j in "${!sibling_last_lines[@]}"; do
    last_line="${sibling_last_lines[$j]}"
    if ! grep -Fxq -- "${sibling_subjects[$j]}" <<< "$origin_all_subjects"; then
        if [[ ! "$last_line" =~ ^\(cherry\ picked\ from\ commit\ ([0-9a-f]{40})\)$ ]]; then
            git cherry-pick -x "${sibling_hashes[$j]}"
            git push
        fi
    fi
done
