#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "用法: $0 <MVS_SDK压缩包.tar.gz>" >&2
  exit 2
fi

archive=$(readlink -f "$1")
case "$(uname -m)" in
  aarch64|arm64)
    library_dir=aarch64
    ;;
  x86_64|amd64)
    library_dir=64
    ;;
  *)
    echo "不支持的CPU架构: $(uname -m)" >&2
    exit 1
    ;;
esac

test -f "$archive"
work_dir=$(mktemp -d)
trap 'rm -rf "$work_dir"' EXIT
tar -xzf "$archive" -C "$work_dir"
test -f "$work_dir/include/MvCameraControl.h"
test -f "$work_dir/lib/$library_dir/libMvCameraControl.so"

sudo mkdir -p /opt/MVS
sudo cp -a "$work_dir"/. /opt/MVS/
# Normalize the archive ownership and access without stripping executable bits.
sudo chown -R root:root /opt/MVS
sudo chmod -R go-w /opt/MVS
sudo find /opt/MVS -type d -exec chmod 0755 {} +

# The vendor archive bundles an old libusb with the same SONAME as Ubuntu's
# system library. Exposing it through ldconfig breaks unrelated consumers such
# as PCL because the bundled copy lacks newer libusb symbols. MVS is compatible
# with Ubuntu's ABI-compatible system libusb, so retain the vendor copy only as
# an offline fallback outside the dynamic linker search paths.
disabled_library_dir="/opt/MVS/vendor-libs-disabled/$library_dir"
sudo mkdir -p "$disabled_library_dir"
sudo find "/opt/MVS/lib/$library_dir" -maxdepth 1 \
  \( -type f -o -type l \) -name 'libusb-1.0.so*' \
  -exec mv -f -t "$disabled_library_dir" {} +
printf '%s\n' \
  "/opt/MVS/lib/$library_dir" \
  "/opt/MVS/lib/$library_dir/ThirdParty" | \
  sudo tee /etc/ld.so.conf.d/hikrobot-mvs.conf >/dev/null
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", MODE="0660", GROUP="video", TAG+="uaccess"' | \
  sudo tee /etc/udev/rules.d/80-agribot-hikrobot-usb.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
sudo ldconfig

echo "MVS SDK已安装到/opt/MVS；未安装厂商网络和开机服务。"
