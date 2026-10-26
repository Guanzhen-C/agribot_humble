"""Pure file-format checks for the offline full-detail Unity display pipeline."""
import importlib.util
from pathlib import Path
import struct

import pytest


spec = importlib.util.spec_from_file_location('full_detail_preview',
    Path(__file__).parents[1] / 'scripts/prepare_full_detail_preview.py')
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def test_preview_records_keep_geometry_colour_and_link_names(tmp_path):
    record = (b'base_link', struct.pack('<4fI', .2, .4, .6, 1, 1),
              struct.pack('<9f', 0, 0, 0, 1, 0, 0, 0, 1, 0))
    path = tmp_path / 'preview.bytes'
    stats = preview.save(path, [record])
    assert preview.records(path) == [record]
    assert stats['triangles'] == 1
    assert stats['meshGroups'] == 1
    assert stats['minimumRos'] == [0, 0, 0]
    assert stats['maximumRos'] == [1, 1, 0]
    assert stats['previewSha256'] == preview.digest(path)


def test_truncated_preview_is_not_published(tmp_path):
    path = tmp_path / 'bad.bytes'
    path.write_bytes(b'AFM1' + struct.pack('<II', 1, 4) + b'body' + struct.pack('<4fI', 1, 1, 1, 1, 1))
    with pytest.raises(ValueError, match='Invalid preview chunk'):
        preview.records(path)


def test_unbounded_group_is_rejected(tmp_path):
    path = tmp_path / 'bad.bytes'
    path.write_bytes(b'AFM1' + struct.pack('<II', 1, 4) + b'body' + struct.pack('<4fI', 1, 1, 1, 1, 300001))
    with pytest.raises(ValueError, match='Invalid preview chunk'):
        preview.records(path)
