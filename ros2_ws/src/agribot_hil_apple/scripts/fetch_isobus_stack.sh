#!/usr/bin/env bash
set -euo pipefail
revision=02f3fe910de5fdc73e2014394c2e80900490b9cc
destination="${AGISOSTACK_ROOT:-$HOME/.cache/agribot/isobus/AgIsoStack-plus-plus}"
if [[ ! -e "$destination" ]]; then
  git clone https://github.com/Open-Agriculture/AgIsoStack-plus-plus.git "$destination"
fi
if [[ -n "$(git -C "$destination" status --porcelain)" ]]; then
  printf 'Refusing a modified stack checkout.\n' >&2
  exit 2
fi
git -C "$destination" cat-file -e "$revision^{commit}" 2>/dev/null || \
  git -C "$destination" fetch origin "$revision"
git -C "$destination" checkout --detach "$revision"
printf '%s\n' "$destination"
