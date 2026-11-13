#!/usr/bin/env bash
# Fails when a tracked file matches the pattern below: references that do not belong in this repository.
#
#   tools/check_references.sh
#
# tools/reference_allowlist.txt lists the hits that may stay: one line per hit, "<path>:<extended regex>", where the
# regex must match the hit's line. Lines starting with "#" and empty lines are ignored. The check itself and its
# allowlist are not searched.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

pattern='armarx|armar[67]|axii|h2t|kit\.edu|/cvhci|/lsdf|/var/tmp|armar-user|visionx|armem|dschneider'
allowlist=tools/reference_allowlist.txt

hits=$(git grep -nIiE "$pattern" -- . ':!tools/check_references.sh' ":!$allowlist" || true)

remaining=""
while IFS= read -r hit; do
  [[ -z "$hit" ]] && continue
  path=${hit%%:*}
  rest=${hit#*:}
  text=${rest#*:}
  allowed=0
  if [[ -f "$allowlist" ]]; then
    while IFS= read -r entry; do
      [[ -z "$entry" || "$entry" == \#* ]] && continue
      entry_path=${entry%%:*}
      entry_regex=${entry#*:}
      if [[ "$path" == "$entry_path" ]] && grep -qiE -- "$entry_regex" <<<"$text"; then
        allowed=1
        break
      fi
    done <"$allowlist"
  fi
  if [[ $allowed -eq 0 ]]; then
    remaining+="$hit"$'\n'
  fi
done <<<"$hits"

if [[ -n "$remaining" ]]; then
  echo "References that must not be in this repository (pattern: $pattern):" >&2
  printf '%s' "$remaining" >&2
  echo "Remove them, or add them to $allowlist if they must stay." >&2
  exit 1
fi
echo "No references found."
