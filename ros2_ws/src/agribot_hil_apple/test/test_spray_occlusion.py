from pathlib import Path
import shutil
import subprocess

import pytest


def test_native_spray_depth_clipping(tmp_path):
    compiler = shutil.which('g++')
    include = Path('/usr/include/ignition/math6')
    if not compiler or not include.is_dir():
        pytest.skip('Native Gazebo geometry headers are optional')
    root = Path(__file__).resolve().parents[1]
    target = tmp_path/'spray_occlusion_test'
    subprocess.run([compiler, '-std=c++17', '-I'+str(include), '-I'+str(root/'plugins'),
                    str(root/'test/spray_occlusion_test.cpp'), '-o', str(target)], check=True)
    subprocess.run([str(target)], check=True)
