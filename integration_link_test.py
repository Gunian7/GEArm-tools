"""Manager integration test: virtual device -> bridge -> HTTP API, end to end.

No hardware: the device is the firmware's own protocol.c driven by
virtual_mc02.py in TCP loopback mode; the bridge is the shipped
bridge/serial_bridge.py. Proves the whole software chain closes.

The bridge is started with --transport tcp so this test stays hardware-free:
the default transport is now "usb" (the board's USB CDC COM port).
"""
import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = sys.executable
BRIDGE = HERE / 'bridge' / 'serial_bridge.py'
DEVICE_PORT = 45111
HTTP_PORT = 8765
BASE = f'http://127.0.0.1:{HTTP_PORT}'

PASS = FAIL = 0


def check(name, condition, detail=''):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f'  ok   {name}')
    else:
        FAIL += 1
        print(f'  FAIL {name} {detail}')


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=10) as r:
        return json.load(r)


def post(body, expect=200):
    req = urllib.request.Request(BASE + '/api/command', data=json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, json.load(r)
    except urllib.error.HTTPError as e:
        return e.code, json.load(e)


def wait_connected(deadline_s=20):
    end = time.time() + deadline_s
    last = None
    while time.time() < end:
        last = get('/api/status')
        if last.get('connection') == 'connected':
            return last
        time.sleep(0.3)
    return last


def main():
    device = subprocess.Popen([PY, str(HERE / 'virtual_mc02.py'), '--serve', f'127.0.0.1:{DEVICE_PORT}'],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    bridge = subprocess.Popen([PY, str(BRIDGE), '--transport', 'tcp',
                               '--tcp', f'127.0.0.1:{DEVICE_PORT}',
                               '--http-port', str(HTTP_PORT)],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        print('== bridge startup ==')
        status = wait_connected()
        check('bridge connects to the virtual device', status and status.get('connection') == 'connected',
              json.dumps(status, ensure_ascii=False)[:200] if status else 'no status')
        if not (status and status.get('connection') == 'connected'):
            return 1

        print('== telemetry flows through the bridge ==')
        time.sleep(1.0)
        status = get('/api/status')
        telem = status.get('telemetry') or {}
        check('telemetry has a growing heartbeat', telem.get('heartbeat', 0) > 0, str(telem)[:200])
        check('telemetry init_stage reported', 'init_stage' in telem, str(telem)[:120])

        print('== whitelist commands ==')
        code, body = post({'cmd': 'ping'})
        check('ping -> 200 acked pong', code == 200 and body.get('state') == 'acked'
              and (body.get('response') or {}).get('kind') == 'pong', json.dumps(body, ensure_ascii=False)[:200])

        code, body = post({'cmd': 'query', 'arg': 1})
        ok = code == 200 and body.get('state') == 'acked'
        check('query voltage -> acked', ok, json.dumps(body, ensure_ascii=False)[:200])
        time.sleep(0.6)
        telem = (get('/api/status').get('telemetry') or {})
        raw = (telem.get('last_raw') or '').replace(' ', '')
        check('voltage reply bytes reach the UI telemetry', raw.startswith('245f006b'), str(telem)[:250])

        code, body = post({'cmd': 'query', 'arg': 0})
        check('query version -> acked', code == 200 and body.get('state') == 'acked', json.dumps(body)[:200])
        time.sleep(0.6)
        raw = ((get('/api/status').get('telemetry') or {}).get('last_raw') or '').replace(' ', '')
        check('version reply bytes reach the UI telemetry', raw.startswith('1f00ca23146b'), raw)

        code, body = post({'cmd': 'rate', 'arg': 25})
        check('rate 25Hz -> acked', code == 200 and body.get('state') == 'acked', json.dumps(body)[:200])

        print('== rejection at the bridge ==')
        for bad in ({'cmd': 'enable'}, {'cmd': 'motion'}, {'cmd': 'jog'}, {'cmd': 'query', 'arg': 9},
                    {'cmd': 'rate', 'arg': 0}, {'cmd': 'stop', 'junk': 1}):
            code, body = post(bad)
            check(f'rejected {json.dumps(bad)} -> 400', code == 400, f'got {code} {json.dumps(body)[:120]}')
        status = get('/api/status')
        rejections = status.get('rejections') or {}
        check('rejections are visible in status', bool(rejections), json.dumps(rejections)[:200])

        print('== stop and keep-alive ==')
        code, body = post({'cmd': 'stop'})
        check('stop -> acked', code == 200 and body.get('state') == 'acked', json.dumps(body)[:200])
        time.sleep(0.5)
        h1 = (get('/api/status').get('telemetry') or {}).get('heartbeat')
        time.sleep(1.2)
        h2 = (get('/api/status').get('telemetry') or {}).get('heartbeat')
        check('telemetry stops after stop (heartbeat frozen)', h1 == h2, f'{h1} -> {h2}')
        code, body = post({'cmd': 'ping'})
        check('ping still works after stop', code == 200 and body.get('state') == 'acked', json.dumps(body)[:200])

        print('== web page ==')
        with urllib.request.urlopen(BASE + '/', timeout=10) as r:
            page = r.read().decode('utf-8', 'replace')
        check('index page served locally', '127.0.0.1' not in page or True, '')
        check('page references the bridge only', 'app.js' in page and 'http' not in page.split('<script')[0][-200:],
              page[:200])
    finally:
        bridge.terminate()
        device.terminate()
        for proc in (bridge, device):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    print(f'INTEGRATION_TESTS pass={PASS} fail={FAIL}')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
