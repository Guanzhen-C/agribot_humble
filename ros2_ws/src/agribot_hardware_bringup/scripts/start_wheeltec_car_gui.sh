#!/bin/sh
set -eu

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/sunrise/.Xauthority}"

lock_dir="/run/user/$(id -u)"
mkdir -p "$lock_dir"
exec 9>"$lock_dir/wheeltec-car-gui.lock"
if ! /usr/bin/flock -n 9; then
    exit 0
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec /usr/bin/python3 "$script_dir/wheeltec_car_gui.py" "$@"
