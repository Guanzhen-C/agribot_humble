from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


@pytest.mark.parametrize('role', ['pc', 'rdk'])
def test_large_images_use_sufficient_buffers_without_ip_fragmentation(role):
    path = Path(__file__).resolve().parents[1] / f'config/dds_{role}.xml'
    root = ET.parse(path)
    namespace = {'d': 'https://cdds.io/config'}
    general = root.find('d:Domain/d:General', namespace)
    assert general.find('d:MaxMessageSize', namespace).text == '1400 B'
    assert general.find('d:FragmentSize', namespace).text == '1200 B'
    buffer = root.find('d:Domain/d:Internal/d:SocketReceiveBufferSize', namespace)
    assert buffer.attrib['min'] == '8 MiB'
    assert general.find('d:AllowMulticast', namespace).text == 'false'
