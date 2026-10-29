"""Build the host-side protocol DLL and run the hardware-free end-to-end link test.

Never touches hardware: no flashing, no JLink, no real serial port.
"""
import hashlib
import json
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent
HOST_CC = 'F:/Tool/MATLAB R2025a(64bit)/toolbox/sldrt/clang/win64/clang.exe'
BUILD = ROOT / 'build'
DLL = BUILD / 'protocol.dll'

result = {'dll_build': 'not_run', 'e2e': 'not_run', 'hardware': 'not_connected'}


def run(cmd):
    p = subprocess.run([str(x) for x in cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, errors='replace')
    if p.returncode:
        raise RuntimeError(f'exit {p.returncode}: {cmd[0]}\n{p.stdout}')
    return p.stdout


try:
    BUILD.mkdir(exist_ok=True)
    # 32-bit host executable (the shipped host_tests pattern); Python drives it
    # over stdin/stdout, so no DLL and no 64-bit linker is needed.
    out = run([HOST_CC, '--target=i686-w64-windows-gnu', '--sysroot=C:/MinGW', '-B', 'C:/MinGW/bin',
               '-ffreestanding', '-nostdlib', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT),
               str(ROOT / 'protocol.c'), str(ROOT / 'virtual_mc02.c'), str(ROOT / 'host_entry.c'),
               '-L', 'C:/MinGW/lib', '-lmsvcrt', '-Wl,-e,_mainCRTStartup',
               '-o', str(BUILD / 'virtual_mc02.exe')])
    result['exe_build'] = 'passed'
    result['exe_log'] = out

    # The exe must answer a status query before any link test runs.
    verify = run([sys.executable, '-c',
                  'import sys;sys.path.insert(0,r"%s");'
                  'from virtual_mc02 import VirtualMcU;'
                  'import struct;'
                  'm=VirtualMcU();'
                  'assert m._command("S")==bytes(8), "status shape";'
                  'first=m._command("T",struct.pack("<I",1000));'
                  'assert len(first)==46, ("bad first frame", len(first));'
                  'm.close();print("EXE_OK")' % str(ROOT)])
    result['exe_selftest'] = verify.strip()

    e2e_out = run([sys.executable, str(ROOT / 'e2e_link_test.py')])
    result['e2e_output'] = e2e_out
    m = re.search(r'E2E_TESTS pass=(\d+) fail=(\d+)', e2e_out)
    if not m:
        raise RuntimeError('e2e summary missing\n' + e2e_out)
    result['e2e_pass'] = int(m.group(1))
    result['e2e_fail'] = int(m.group(2))
    if result['e2e_fail']:
        raise RuntimeError(f"e2e failed pass={result['e2e_pass']} fail={result['e2e_fail']}")
    result['e2e'] = 'passed'

    names = ['protocol.c', 'protocol.h', 'virtual_mc02.c', 'virtual_mc02.py', 'e2e_link_test.py']
    result['source_sha256'] = {n: hashlib.sha256((ROOT / n).read_bytes()).hexdigest() for n in names}
    result['exe_sha256'] = hashlib.sha256((BUILD / 'virtual_mc02.exe').read_bytes()).hexdigest()
    result['tested'] = ['firmware protocol.c compiled to a host exe and driven over a real byte stream',
                        'pc_service.py client crc/Parser/unpack_telemetry on the receiving end',
                        'virtual CAN driver answers only the three recorded read-only replies',
                        'TCP loopback stand-in used by the bridge transport']
    result['untested'] = ['真实 USB CDC 链路与电平', '真实 CAN 回包', '硬件时序',
                          'pc_service.py --selftest 的 Python 虚拟设备与本 C 虚拟设备行为一致性未逐项比对']
except Exception as exc:
    result['error'] = str(exc)
finally:
    (ROOT / 'e2e-result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')

print(json.dumps({k: result.get(k) for k in ('dll_build', 'dll_selftest', 'e2e', 'e2e_pass', 'e2e_fail')},
                 indent=2, ensure_ascii=False))
sys.exit(1 if 'error' in result else 0)
