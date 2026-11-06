#!/usr/bin/env bash
# Build the pinned external Gazebo dependency without changing the main install.
set -euo pipefail
package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${HIL_OVERLAY_ROOT:-$HOME/.local/state/agribot-hil-20261105}"
repo="$root/external/IFRA_LinkAttacher"
revision=b056289ba93ccb549db98926dcbb9679642d0c8d
if [[ ! -d "$repo/.git" ]]; then
  git clone --no-checkout https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git "$repo"
  git -C "$repo" checkout --detach "$revision"
fi
if [[ "$(git -C "$repo" rev-parse HEAD)" != "$revision" ]]; then
  printf 'Dependency revision differs from the pinned revision; refusing to overwrite.\n' >&2
  exit 1
fi
patch="$package/patches/ifra_multi_fixture.patch"
if git -C "$repo" apply --reverse --check "$patch"; then
  printf 'Attachment patch already applied.\n'
else
  git -C "$repo" apply --check "$patch"
  git -C "$repo" apply "$patch"
fi
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin colcon --log-base "$root/colcon_log" build \
  --base-paths "$repo" --packages-select linkattacher_msgs ros2_linkattacher \
  --executor sequential --build-base "$root/build" --install-base "$root/install"
