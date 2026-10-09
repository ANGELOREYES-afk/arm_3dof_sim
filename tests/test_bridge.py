"""Offline bridge checks against im6014Development's packet-level DemoSerial. No serial device.

    python3 tests/test_bridge.py
"""
import base64
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'bridge'))
import im6014  # noqa: E402
from arm_session import ArmSession  # noqa: E402

LIB = im6014.load()
CFG = json.loads((ROOT / 'config' / 'arm.json').read_text())
MANIFEST = json.loads((ROOT / 'web' / 'meshes' / 'manifest.json').read_text())
TMP = tempfile.TemporaryDirectory()


def session(cfg=CFG):
    d = Path(TMP.name)
    for f in d.glob('*.json'):
        f.unlink()
    return ArmSession(cfg, LIB, MANIFEST, dict(real=d / 'real.json', demo=d / 'demo.json'))


def wait(session, pred, timeout=3.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = session.snapshot()
        if pred(s):
            return s
        time.sleep(.02)
    raise AssertionError(f"timed out; state={s['state']!r} reason={s['reason']!r}")


def heartbeat(session, stop):
    while not stop.is_set():
        session.ping()
        time.sleep(.05)


def test_lifecycle():
    s = session()
    hb = threading.Event()
    threading.Thread(target=heartbeat, args=(s, hb), daemon=True).start()
    try:
        s.request('arm', supported=True)
        wait(s, lambda d: 'Rejected' in ''.join(e[1] for e in d['events']))
        s.request('connect', demo=True)
        d = wait(s, lambda d: d['state'] == 'Timeout — disarmed')
        assert d['connected'] and not d['zeroed'], d
        s.request('ack')
        wait(s, lambda d: d['state'] == 'Ready · Stop')
        s.request('arm', supported=True)
        wait(s, lambda d: 'Calibrate before arming' in d['reason'])
        # Manual reference: the arm is declared to be at the calibration pose (the CAD pose).
        s.request('home', supported=True)
        wait(s, lambda d: d['zeroed'] and d['state'] == 'Ready · Stop')
        s.request('arm', supported=False)
        wait(s, lambda d: 'acknowledgment' in d['reason'])
        s.request('arm', supported=True)
        d = wait(s, lambda d: d['armed'])
        pose = CFG['calibration_pose_deg']
        assert max(abs(a - b) for a, b in zip(d['q_cmd'], pose)) < 1e-6, d['q_cmd']

        # Without the deadman, targets are ignored: the arm holds.
        for _ in range(20):
            s.set_target([0.5, 0.5, 0.5], deadman=False)
            time.sleep(.01)
        d = s.snapshot()
        assert d['state'] == 'Armed · holding' and max(abs(a - b) for a, b in zip(d['q_cmd'], pose)) < 1e-6, d['q_cmd']

        # With the deadman, q_cmd ramps at max_speed and the simulated motors follow.
        # (The demo joints have physical stops; these targets stay clear of them.)
        goal = [30., 20., 400.]  # J3 past its 390 deg limit -> clamped
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.:
            s.set_target([math.radians(a) for a in goal], deadman=True)
            d = s.snapshot()
            if time.monotonic() - t0 < .5:
                # J2 max 30 deg/s: after <0.5 s it cannot be past ~15 deg
                assert abs(d['q_cmd'][1]) <= 30 * (time.monotonic() - t0) + 2, d['q_cmd']
            time.sleep(.01)
        d = s.snapshot()
        assert d['state'] == 'Armed · moving', d['state']
        assert abs(d['q_cmd'][0] - 30) < .5 and abs(d['q_cmd'][1] - 20) < .5, d['q_cmd']
        assert abs(d['q_cmd'][2] - 390) < .5, d['q_cmd']  # clamped to the limit
        meas = [f['q_deg'] for f in d['feedback']]
        assert all(abs(m - c) < 5 for m, c in zip(meas, d['q_cmd'])), (meas, d['q_cmd'])
        assert d['tx'] == d['rx'] and d['crc_errors'] == 0 and d['reply_timeouts'] == 0, d
        print(f"  tracking ok: cmd {d['q_cmd']} meas {[round(m, 2) for m in meas]} · loop {d['loop_hz']:.0f} Hz")

        # Deadman released: hold where we are.
        for _ in range(30):
            s.set_target([0., 0., 6.], deadman=False)
            time.sleep(.01)
        assert abs(s.snapshot()['q_cmd'][0] - 30) < .5

        # Browser heartbeat loss -> Stop + disarm.
        hb.set()
        d = wait(s, lambda d: d['state'].startswith('Fault'), timeout=2)
        assert not d['armed'] and 'heartbeat' in d['reason'], d
        assert all(f['mode'] == 0 for f in d['feedback']), d['feedback']

        # Recover: a fault needs an explicit acknowledgment, then re-arm at the measured pose.
        hb.clear()
        threading.Thread(target=heartbeat, args=(s, hb), daemon=True).start()
        s.request('check')
        wait(s, lambda d: 'Acknowledge the fault' in d['reason'])
        s.request('ack')
        wait(s, lambda d: d['state'] == 'Ready · Stop')
        s.request('arm', supported=True)
        d = wait(s, lambda d: d['armed'])
        assert abs(d['q_cmd'][0] - 30) < 2, d['q_cmd']  # re-arms where the arm actually is
        s.request('stop')
        d = wait(s, lambda d: d['state'] == 'Stopped · re-check required')
        assert not d['armed'] and all(f['mode'] == 0 for f in d['feedback'])
        s.request('disconnect')
        wait(s, lambda d: d['state'] == 'Disconnected')
    finally:
        hb.set()
        s.close()
    print('lifecycle: ok')


def test_tracking_trip():
    cfg = json.loads(json.dumps(CFG))
    cfg['safety']['max_tracking_error_deg'] = 1
    cfg['joints'][1]['max_speed_dps'] = 400
    s = session(cfg)
    hb = threading.Event()
    threading.Thread(target=heartbeat, args=(s, hb), daemon=True).start()
    try:
        s.request('connect', demo=True)
        wait(s, lambda d: d['state'] == 'Timeout — disarmed')
        s.request('ack')
        wait(s, lambda d: d['state'] == 'Ready · Stop')
        s.request('home', supported=True)
        wait(s, lambda d: d['zeroed'])
        s.request('arm', supported=True)
        wait(s, lambda d: d['armed'])
        for _ in range(5):  # release the deadman once after arming
            s.set_target([0, 0, 0], deadman=False)
            time.sleep(.02)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1 and s.snapshot()['armed']:
            s.set_target([0, 1.5, 0], deadman=True)
            time.sleep(.01)
        d = wait(s, lambda d: d['state'].startswith('Fault'))
        assert 'tracking error' in d['reason'], d['reason']
    finally:
        hb.set()
        s.close()
    print('tracking-error trip: ok')


def test_gravity_sign():
    s = session()
    try:
        a = MANIFEST['analytic']
        # Upper arm and forearm horizontal, reaching forward (closed-form angles 90 and 0).
        q = [0., (math.pi / 2 - a['offset'][1]) * a['sign'][1], (0 - a['offset'][2]) * a['sign'][2]]
        g = s._gravity(q)
        assert g[0] == 0 and g[1] * a['sign'][1] < 0 and g[2] * a['sign'][2] < 0, g  # pushes back up
        up = [0., -a['offset'][1] * a['sign'][1], -a['offset'][2] * a['sign'][2]]
        assert max(abs(x) for x in s._gravity(up)) < 1e-9  # straight up: nothing to hold
    finally:
        s.close()
    print(f'gravity model: ok (arm straight out needs {abs(g[1]):.2f} / {abs(g[2]):.2f} Nm at J2/J3)')


def ws_client(port, origin=None):
    sock = socket.create_connection(('127.0.0.1', port), timeout=3)
    key = base64.b64encode(os.urandom(16)).decode()
    hdr = f'GET /ws HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n' \
          f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n'
    if origin:
        hdr += f'Origin: {origin}\r\n'
    sock.sendall((hdr + '\r\n').encode())
    f = sock.makefile('rb')
    status = f.readline().decode()
    while f.readline() not in (b'\r\n', b''):
        pass
    return sock, f, status


def ws_send(sock, obj):
    data = json.dumps(obj).encode()
    mask = os.urandom(4)
    assert len(data) < 126
    sock.sendall(struct.pack('!BB', 0x81, 0x80 | len(data)) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))


def ws_recv(f):
    b1, b2 = f.read(2)
    n = b2 & 0x7f
    if n == 126:
        n = struct.unpack('!H', f.read(2))[0]
    elif n == 127:
        n = struct.unpack('!Q', f.read(8))[0]
    return json.loads(f.read(n))


def test_websocket():
    import subprocess
    port = 8799
    proc = subprocess.Popen([sys.executable, str(ROOT / 'bridge' / 'arm_bridge.py'), '--port', str(port)],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        for _ in range(50):
            try:
                socket.create_connection(('127.0.0.1', port), timeout=.2).close()
                break
            except OSError:
                time.sleep(.1)
        _, _, status = ws_client(port, origin='https://evil.example')
        assert ' 403 ' in status, status
        sock, f, status = ws_client(port, origin=f'http://127.0.0.1:{port}')
        assert ' 101 ' in status, status
        hello = ws_recv(f)
        assert hello['type'] == 'hello' and hello['config']['joints'][0]['id'] == 0
        ws_send(sock, dict(type='cmd', action='connect', demo=True))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            msg = ws_recv(f)
            if msg['type'] == 'state' and msg['state'] == 'Timeout — disarmed':
                break
        else:
            raise AssertionError('no demo connect over WebSocket')
        assert msg['controller'] and not msg['armed']
        # A second page is an observer and cannot command, but may stop.
        sock2, f2, _ = ws_client(port)
        ws_recv(f2)
        ws_send(sock2, dict(type='cmd', action='disconnect'))
        time.sleep(.3)
        while (m := ws_recv(f2))['type'] != 'state':
            pass
        assert m['observer'] and m['connected'], m
        sock2.close()
        sock.close()
    finally:
        proc.terminate()
        out = proc.communicate(timeout=5)[0].decode()
    assert 'Traceback' not in out, out
    print('websocket: ok (origin check, hello, demo connect, observer cannot command)')


if __name__ == '__main__':
    test_gravity_sign()
    test_lifecycle()
    test_tracking_trip()
    test_websocket()
    print('ALL BRIDGE TESTS PASSED')
