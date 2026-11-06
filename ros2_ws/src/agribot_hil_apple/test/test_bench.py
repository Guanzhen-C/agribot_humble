from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
NS = {'d': 'https://cdds.io/config'}


def test_wired_profiles_cannot_select_wireless():
    for role, address in [('pc', '192.168.1.1'), ('rdk', '192.168.1.102')]:
        domain = ET.parse(ROOT / 'config' / f'dds_{role}.xml').getroot().find('d:Domain', NS)
        interfaces = domain.findall('d:General/d:Interfaces/d:NetworkInterface', NS)
        assert [element.attrib for element in interfaces] == [{'address': address}]
        assert domain.find('d:General/d:AllowMulticast', NS).text == 'false'
        assert domain.find('d:Discovery/d:ParticipantIndex', NS).text == 'auto'
        assert domain.find('d:Discovery/d:MaxAutoParticipantIndex', NS).text == '119'
        assert {p.attrib['address'] for p in domain.findall('d:Discovery/d:Peers/d:Peer', NS)} == {
            '192.168.1.1', '192.168.1.102'}


def test_bench_usb_devices_are_distinct_and_motion_defaults_off():
    import yaml
    data = yaml.safe_load((ROOT / 'config/bench.yaml').read_text())
    assert data['can']['motion_authorized'] is False
    assert data['can']['can_fd'] is False and data['can']['extended'] is False
    assert data['can']['bitrate'] == 1000000
    assert data['can']['topology'] == 'hpm6750_native_bridge'
    assert data['can']['hpm6750_enabled'] is True
    assert data['pc']['usb_can_serial'] != data['rdk']['usb_can_serial']
    assert data['rdk']['can_transport'] == 'zqwl_cdc'
    assert data['rdk']['can_interface_observed'] is None
    assert data['rdk']['usb_can_serial'] == '966960660237'


def test_boot_network_profile_changes_only_buffer_ceilings():
    lines = (ROOT / 'config/99-agribot-hil-network.conf').read_text().splitlines()
    values = dict(line.split('=', 1) for line in lines if line and not line.startswith('#'))
    assert {key.strip(): int(value) for key, value in values.items()} == {
        'net.core.rmem_max': 16777216, 'net.core.wmem_max': 16777216}
