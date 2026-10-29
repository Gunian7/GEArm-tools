"""Extract the firmware DH table and build the host kinematics exe.

The DH values come from GEArm-H723-Firmware/Core/Src/robot.c at build time, so
the host FK/IK can never silently drift from the firmware table. If the table
cannot be parsed, the build fails loudly.
"""
import hashlib
import json
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent
FW = ROOT.parent / 'GEArm-H723-Firmware' / 'Core' / 'Src' / 'robot.c'
HOST_CC = 'F:/Tool/MATLAB R2025a(64bit)/toolbox/sldrt/clang/win64/clang.exe'
BUILD = ROOT / 'build'
EXE = BUILD / 'kinematics_host.exe'
PY = sys.executable

result = {'dh_extract': 'not_run', 'exe_build': 'not_run', 'roundtrip': 'not_run'}


def run(cmd):
    p = subprocess.run([str(x) for x in cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, errors='replace')
    if p.returncode:
        raise RuntimeError(f'exit {p.returncode}: {cmd[0]}\n{p.stdout}')
    return p.stdout


try:
    BUILD.mkdir(exist_ok=True)
    src = FW.read_text(encoding='utf-8', errors='replace')
    m = re.search(r'const\s+float\s+D_H\[6\]\[4\]\s*=\s*\{(.*?)\};', src, re.S)
    if not m:
        raise RuntimeError('D_H table not found in robot.c')
    rows = re.findall(r'\{([^}]*)\}', m.group(1))
    if len(rows) != 6:
        raise RuntimeError(f'expected 6 DH rows, found {len(rows)}')
    values = []
    for row in rows:
        cells = [c.strip() for c in row.split(',') if c.strip()]
        if len(cells) != 4:
            raise RuntimeError(f'DH row must have 4 cells: {row!r}')
        values.append(cells)
    header = ['/* Generated from GEArm-H723-Firmware/Core/Src/robot.c. Do not edit. */',
              '#ifndef DH_FROM_FIRMWARE_H', '#define DH_FROM_FIRMWARE_H', '',
              '#define M_PI 3.14159265358979323846', '']
    header.append('static const double D_H[6][4] = {')
    for row in values:
        header.append('  {' + ', '.join(row) + '},')
    header.append('};')
    header.append('')
    header.append('#endif')
    (ROOT / 'dh_from_firmware.h').write_text('\n'.join(header) + '\n', encoding='utf-8')
    result['dh_extract'] = 'passed'
    result['dh_source_sha256'] = hashlib.sha256(FW.read_bytes()).hexdigest()
    result['dh_values'] = values

    run([HOST_CC, '--target=i686-w64-windows-gnu', '--sysroot=C:/MinGW', '-B', 'C:/MinGW/bin',
         '-ffreestanding', '-nostdlib', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT),
         str(ROOT / 'kinematics_host.c'), str(ROOT / 'host_entry.c'),
         '-L', 'C:/MinGW/lib', '-lmsvcrt', '-Wl,-e,_mainCRTStartup',
         '-o', str(EXE)])
    result['exe_build'] = 'passed'
    result['exe_sha256'] = hashlib.sha256(EXE.read_bytes()).hexdigest()

    out = run([PY, '-c',
               'import sys,struct;sys.path.insert(0,r"%s");'
               'from kinematics import Kinematics;'
               'k=Kinematics();'
               'fk=k.fk([0,0,0,0,0,0]);'
               'assert len(fk["tcp"])==3 and len(fk["chain"])==6, fk;'
               'ik=k.ik(fk["tcp"],[0]*6);'
               'assert ik["residual"]<1e-3, ik;'
               'rt=k.roundtrip(80);'
               'assert rt["failures"]==0, ("ik failures",rt);'
               'assert rt["worst_mm"]<1e-3, ("roundtrip error too large",rt);'
               'print("KIN_OK worst_mm=%%.6f failures=%%d"%%(rt["worst_mm"],rt["failures"]))' % str(ROOT)])
    result['roundtrip'] = 'passed'
    result['roundtrip_output'] = out.strip()
    m2 = re.search(r'worst_mm=([0-9.eE+-]+)', out)
    if m2:
        result['roundtrip_max_error_mm'] = float(m2.group(1))
    m3 = re.search(r'failures=(\d+)', out)
    if m3:
        result['roundtrip_failures'] = int(m3.group(1))
    result['tested'] = ['DH table extracted from firmware robot.c at build time',
                        'FK standard DH chain in host C',
                        'IK damped least squares with multi-start seeds',
                        'FK->IK->FK round trip over 80 random reachable poses']
    result['untested'] = ['与固件 robot_kinematics.c 解析逆解的一致性（需硬件或固件侧运行）',
                          '真实连杆尺寸/减速比/零位适配', '关节限位（host 侧未建模 min/max）']
except Exception as exc:
    result['error'] = str(exc)
finally:
    (ROOT / 'kinematics-result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False),
                                                 encoding='utf-8')

print(json.dumps({k: result.get(k) for k in ('dh_extract', 'exe_build', 'roundtrip',
                                             'roundtrip_max_error_mm')}, ensure_ascii=False))
sys.exit(1 if 'error' in result else 0)
