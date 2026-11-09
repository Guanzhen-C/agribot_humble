#!/usr/bin/env bash
set -eo pipefail
package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${ISOBUS_HIL_STATE:-$HOME/.local/state/agribot-isobus-hil-20261109}"
stack="$(bash "$package/scripts/fetch_isobus_stack.sh")"
generator='Unix Makefiles'
command -v ninja >/dev/null && generator=Ninja
/usr/bin/cmake -S "$package/isobus" -B "$root/build-host" -G "$generator" \
  -DAGISOSTACK_ROOT="$stack" -DCMAKE_BUILD_TYPE=RelWithDebInfo
/usr/bin/cmake --build "$root/build-host" --parallel 2
if [[ "${1:-}" == --firmware ]]; then
  : "${HPM_SDK_BASE:?Set HPM_SDK_BASE to the existing SDK}"
  : "${GNURISCV_TOOLCHAIN_PATH:?Set GNURISCV_TOOLCHAIN_PATH}"
  /usr/bin/cmake -S "$package/firmware/isobus_tracked" -B "$root/build-hpm" -G "$generator" \
    -DBOARD=hpm6750evk2 -DHPM_BUILD_TYPE=flash_xip -DAGISOSTACK_ROOT="$stack"
  /usr/bin/cmake --build "$root/build-hpm" --parallel 2
fi
