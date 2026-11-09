from pathlib import Path
import subprocess


def test_shared_embedded_work_protocol(tmp_path):
    package = Path(__file__).resolve().parents[1]
    executable = tmp_path / 'work_protocol_test'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(package/'isobus'),
                    str(package/'isobus/work_protocol.c'),
                    str(package/'test/work_protocol_test.c'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
