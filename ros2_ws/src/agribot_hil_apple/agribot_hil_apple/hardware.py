"""Read-only inventory. A network device existing is not proof of CAN traffic."""
import json
from pathlib import Path
import platform
import subprocess


def read(path):
    try:
        return Path(path).read_text().replace('\0', '').strip()
    except (OSError, UnicodeError):
        return None


def inventory(sys_root=Path('/sys')):
    interfaces = []
    for path in sorted((Path(sys_root) / 'class/net').iterdir()):
        if read(path / 'type') != '280':
            continue
        device = (path / 'device').resolve()
        usb = next((p for p in (device, *device.parents) if (p / 'idVendor').is_file()), None)
        interfaces.append({
            'interface': path.name, 'driver': (device / 'driver').resolve().name,
            'state': read(path / 'operstate'),
            'usb_vendor': read(usb / 'idVendor') if usb else None,
            'usb_product': read(usb / 'idProduct') if usb else None,
            'usb_serial': read(usb / 'serial') if usb else None,
        })
    return {'architecture': platform.machine(), 'kernel': platform.release(),
            'board_model': read('/proc/device-tree/model'),
            'os_release': read('/etc/os-release'), 'can_interfaces': interfaces,
            'ros_humble': Path('/opt/ros/humble/setup.bash').is_file()}


def main():
    data = inventory()
    for key, args in [('addresses', ['ip', '-j', 'address']),
                      ('routes', ['ip', '-j', 'route']),
                      ('can_details', ['ip', '-j', '-details', '-statistics', 'link', 'show', 'type', 'can'])]:
        result = subprocess.run(args, check=True, capture_output=True, text=True)
        data[key] = json.loads(result.stdout)
    print(json.dumps(data, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
