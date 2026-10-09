"""Shared-bus transport and control-policy checks, on the demo bus. No serial device.

    python3 tests/test_transport.py

With sidrk-dev/im6014-workbench the bridge uses its exchange, which skips a foreign-ID or corrupt
frame and keeps reading for the requested motor; im6014Development's raises on the first bad
frame. Both report a missing motor as a deadline fault.
"""
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'bridge'))
import im6014  # noqa: E402
from arm_session import ArmSession  # noqa: E402
import demo_motors  # noqa: E402

LIB = im6014.load()
CFG = json.loads((ROOT / 'config' / 'arm.json').read_text())
MANIFEST = json.loads((ROOT / 'web' / 'meshes' / 'manifest.json').read_text())
WORKBENCH = LIB.serial is not None


def wait(s, pred, timeout=4, what=''):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        d = s.snapshot()
        if pred(d):
            return d
        time.sleep(.02)
    raise AssertionError(f"{what}: state={d['state']!r} reason={d['reason']!r}")


def session(cfg=CFG):
    tmp = tempfile.mkdtemp()
    s = ArmSession(cfg, LIB, MANIFEST, dict(real=Path(tmp) / 'r.json', demo=Path(tmp) / 'd.json'))
    stop, deadman = threading.Event(), threading.Event()

    def beat():
        while not stop.is_set():
            s.set_target(s.target or [0., 0., 0.], deadman=deadman.is_set())
            time.sleep(.02)
    threading.Thread(target=beat, daemon=True).start()
    return s, stop, deadman


def ready(s, home=True):
    s.request('connect', demo=True)
    wait(s, lambda d: d['state'].startswith('Timeout'), what='timeout latch')
    s.request('ack')
    wait(s, lambda d: d['state'] == 'Ready · Stop', what='ready')
    if home:
        s.request('home', supported=True)
        wait(s, lambda d: d['zeroed'] and d['verified'] and d['state'] == 'Ready · Stop', what='home')


def interfere(bus, joint_id, frame_fn, count):
    """Make the next `count` replies to joint_id arrive after (or as) a bad frame."""
    original = bus.write
    left = [count]

    def write(packet):
        n = original(packet)
        if packet[2] & 15 == joint_id and left[0] > 0:
            left[0] -= 1
            bus.data = frame_fn(bus.data)
        return n
    bus.write = write


def foreign_first(reply):
    """A valid reply from ID 2 ahead of the real one (late reply / bus echo)."""
    body = bytearray(reply[2:22])
    body[0] = (body[0] & 0xF0) | 2
    body = bytes(body)
    return b'\xfc\xee' + body + LIB.codec.crc32(body).to_bytes(4, 'little') + reply


def corrupt(reply):
    return reply[:-1] + bytes([reply[-1] ^ 1])


def test_foreign_frame_on_shared_bus():
    s, stop, _ = session()
    try:
        ready(s)
        interfere(s.buses[0], 1, foreign_first, 50)
        time.sleep(1.5)
        d = s.snapshot()
        if WORKBENCH:
            assert d['state'] == 'Ready · Stop', (d['state'], d['reason'])
            assert d['crc_errors'] == 0 and d['reply_timeouts'] == 0 and d['rx'] == d['tx'], d
            print('foreign-ID frame: ok (workbench exchange skipped 50 foreign frames, no fault)')
        else:
            assert d['state'].startswith('Fault'), d['state']
            print('foreign-ID frame: faults on im6014Development (expected; use the workbench)')
    finally:
        stop.set()
        s.close()


def test_corrupt_and_missing_replies_are_attributed():
    s, stop, _ = session()
    try:
        ready(s)
        interfere(s.buses[0], 2, corrupt, 10 ** 6)
        d = wait(s, lambda d: d['state'].startswith('Fault'), what='CRC fault')
        assert 'J3 elbow (ID 2)' in d['reason'] and d['crc_errors'] >= 1 and d['reply_timeouts'] >= 1, d
        if WORKBENCH:
            assert d['rejected_frames'] >= d['crc_errors'], d
            packets = s.diagnostics()['packets']['records']
            assert any(r['outcome'] == 'INVALID_FEEDBACK' and r['crc_rejected_frames'] for r in packets), packets[-3:]
        print(f"corrupt reply: ok (fault names J3; crc_errors={d['crc_errors']}, rejected={d['rejected_frames']})")
    finally:
        stop.set()
        s.close()

    s, stop, _ = session()
    try:
        ready(s)
        del s.buses[0].motors[CFG['joints'][1]['id']]  # J2 stops answering (unplugged / unpowered)
        d = wait(s, lambda d: d['state'].startswith('Fault'), what='missing motor')
        assert f"J2 shoulder (ID {CFG['joints'][1]['id']})" in d['reason'] and d['reply_timeouts'] >= 1, d
        print('missing motor: ok (deadline fault names J2)')
    finally:
        stop.set()
        s.close()


def test_acceleration_limit_and_effort_guard():
    # The planner itself, at a fixed time step: speed and acceleration bounds, arrival, no overshoot.
    s, stop, deadman = session()
    try:
        dt = 0.01
        s.lim = s.cfg_lim
        s.q_cmd, s.v_cmd = [0., 0., math.radians(200)], [0., 0., 0.]
        goal = [math.radians(60), math.radians(-30), math.radians(320)]
        trace = []
        for _ in range(1000):
            q, v = s._plan_step(goal, dt)
            for i, j in enumerate(CFG['joints']):
                assert abs(v[i]) <= math.radians(j['max_speed_dps']) + 1e-9, (i, v)
                assert abs(v[i] - s.v_cmd[i]) <= math.radians(j['max_accel_dps2']) * dt + 1e-9, (i, v, s.v_cmd)
                overshoot = -(goal[i] - q[i]) * math.copysign(1, goal[i] - s.q_cmd[i])
                assert overshoot < 1e-4, f'J{i + 1} overshoots by {math.degrees(overshoot):.4f} deg'  # rounding only
            s.q_cmd, s.v_cmd = q, v
            trace.append(q)
        assert max(abs(a - b) for a, b in zip(s.q_cmd, goal)) < 1e-6 and max(map(abs, s.v_cmd)) < 1e-6, (s.q_cmd, s.v_cmd)
    finally:
        stop.set()
        s.close()

    # Live: a 40 deg J1 move with the deadman held arrives, and no faster than its speed limit allows.
    s, stop, deadman = session()
    try:
        ready(s)
        s.request('arm', supported=True)
        wait(s, lambda d: d['armed'], what='armed')
        time.sleep(.1)  # deadman released once
        q0 = list(s.q_cmd)
        s.target = [q0[0] + math.radians(40), q0[1], q0[2]]
        deadman.set()
        t0 = time.monotonic()
        d = wait(s, lambda d: abs(d['q_cmd'][0] - math.degrees(q0[0]) - 40) < .05, 5, 'arrive')
        took = time.monotonic() - t0
        deadman.clear()
        j1 = CFG['joints'][0]
        minimum = 40 / j1['max_speed_dps'] + j1['max_speed_dps'] / j1['max_accel_dps2']  # trapezoid
        assert took >= minimum * 0.9, (took, minimum)
        print(f"acceleration limit: ok (planner bounds hold every step; live 40 deg move took {took:.2f}s, "
              f"trapezoid minimum {minimum:.2f}s)")
    finally:
        stop.set()
        s.close()

    cfg = json.loads(json.dumps(CFG))
    cfg['safety']['effort_limit_nm'] = 0.05
    s, stop, deadman = session(cfg)
    try:
        ready(s)
        s.request('arm', supported=True)
        wait(s, lambda d: d['armed'], what='armed')
        time.sleep(.1)
        q0 = list(s.q_cmd)
        s.target = [q0[0], q0[1] + math.radians(20), q0[2]]
        deadman.set()
        d = wait(s, lambda d: d['effort_limited'], what='effort guard')
        time.sleep(.5)
        moved = abs(s.snapshot()['q_cmd'][1] - math.degrees(q0[1]))
        assert moved < 5, moved
        print(f"effort guard: ok ({d['effort_limited']} > 0.05 Nm; J2 advanced only {moved:.1f} deg of 20)")
    finally:
        stop.set()
        s.close()


def test_invalid_command_while_armed_faults_at_once():
    s, stop, _ = session()
    try:
        ready(s)
        s.request('arm', supported=True)
        wait(s, lambda d: d['armed'], what='armed')
        s.joints[1]['kp'] = 99  # outside the codec's 0-30 bench range: command() raises ValueError
        t0 = time.monotonic()
        d = wait(s, lambda d: d['state'].startswith('Fault'), 1, 'fault')
        assert not d['armed'] and 'Kp' in d['reason'] and all(f['mode'] == 0 for f in d['feedback']), d['reason']
        print(f'invalid command while armed: ok (fault + Stop in {1000 * (time.monotonic() - t0):.0f} ms, not a silent reject)')
    finally:
        stop.set()
        s.close()


def test_verify_catches_a_wrong_calibration():
    s, stop, deadman = session()
    try:
        ready(s, home=False)
        # A saved calibration whose J2 reference is 10 deg off (e.g. the encoder did not report an
        # absolute angle at power-up): verification must refuse it.
        fb = s.snapshot()['feedback']
        truth = [demo_motors.true_angle_deg(s.buses[0], CFG['joints'], i) for i in range(3)]
        truth[1] += 10
        refs = [dict(method='manual', q_ref_deg=q, enc_ref_deg=fb[i]['position_deg'], direction=1) for i, q in enumerate(truth)]
        s._save_refs(refs, 'demo')
        s.request('check')
        d = wait(s, lambda d: d['zeroed'] and d['state'] == 'Ready · Stop', what='loaded')
        assert not d['verified']
        deadman.set()
        s.request('verify', supported=True)
        d = wait(s, lambda d: d['state'].startswith('Verification failed'), 30, 'verify failure')
        assert 'does not match' in d['reason'] and not d['armed'], d['reason']
        print('verify: ok (a 10 deg J2 offset is caught at the stop; motors stopped)')
    finally:
        stop.set()
        s.close()


if __name__ == '__main__':
    print('library:', LIB.name)
    test_foreign_frame_on_shared_bus()
    test_corrupt_and_missing_replies_are_attributed()
    test_acceleration_limit_and_effort_guard()
    test_invalid_command_while_armed_faults_at_once()
    test_verify_catches_a_wrong_calibration()
    print('ALL TRANSPORT TESTS PASSED')
