"""Calibration checks: the pure math, then a full demo calibration against simulated absolute
encoders with hidden mounting offsets and CAD-predicted stops. No serial device.

    python3 tests/test_calibration.py
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
from calibration import ContactSeek, SeekSettings, joint_angle, solve_two_contacts, wrap  # noqa: E402
import demo_motors  # noqa: E402

codec, sim = im6014.load()
CFG = json.loads((ROOT / 'config' / 'arm.json').read_text())
MANIFEST = json.loads((ROOT / 'web' / 'meshes' / 'manifest.json').read_text())


def test_math():
    assert wrap(190) == -170 and wrap(-190) == 170 and wrap(180) == 180 and wrap(540) == 180
    ref = dict(q_ref_deg=-158.65, direction=-1, enc_ref_deg=170.)
    # Encoder wraps past +-180 while the joint moves through its reference: q stays continuous.
    for enc, q in ((170., -158.65), (-175., -158.65 - 15), (150., -158.65 + 20)):
        assert abs(joint_angle(ref, enc) - q) < 1e-9, (enc, joint_angle(ref, enc), q)
    # Two contacts -> reference at the predicted midpoint; span check passes and fails as it should.
    r = solve_two_contacts(1, 100., 160.6, -7.59, 53.11, 4)
    assert 'error' not in r and abs(r['enc_ref_deg'] - 130.3) < 1e-9 and abs(r['q_ref_deg'] - 22.76) < 1e-9
    assert 'error' in solve_two_contacts(1, 100., 180., -7.59, 53.11, 4)
    assert 'wrong order' in solve_two_contacts(-1, 100., 160., -7.59, 53.11, 4)['error']
    # Contact seek on a synthetic joint that stops at 30 deg.
    seek = ContactSeek(start_deg=0., motor_sign=1, cfg=SeekSettings(speed_dps=10))
    pos, t = 0., 0.
    while not seek.result and t < 10:
        pos_new = min(30., seek.target_deg)
        speed = (pos_new - pos) / .01
        pos = pos_new
        seek.step(.01, pos, speed)
        t += .01
    assert seek.result == 'contact' and abs(pos - 30) < 1e-9 and t < 3.8, (seek.result, pos, t)
    # Deadman released: the target holds still.
    held = ContactSeek(start_deg=0., motor_sign=-1)
    held.step(.5, 0., 0., active=False)
    assert held.target_deg == 0.
    print('math: ok')


def heartbeat(session, stop, deadman):
    while not stop.is_set():
        session.set_target([0., 0., 0.], deadman=deadman.is_set())
        time.sleep(.02)


def wait(session, pred, timeout, what):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        d = session.snapshot()
        if pred(d):
            return d
        time.sleep(.02)
    raise AssertionError(f"waiting for {what}: state={d['state']!r} reason={d['reason']!r} cal={d['calibration']['message']!r}")


def truth_deg(session, i):
    """The simulated joint's true URDF angle, from the demo's hidden mounting angle."""
    pos = math.degrees(session.buses[i].states[CFG['joints'][i]['id']]['pos'])
    return (pos - demo_motors.MOUNTING_DEG[i]) * CFG['joints'][i]['direction']


def run_session(cfg, cal_dir, manifest=MANIFEST):
    s = ArmSession(cfg, codec, sim, manifest, dict(real=cal_dir / 'real.json', demo=cal_dir / 'demo.json'))
    stop, deadman = threading.Event(), threading.Event()
    threading.Thread(target=heartbeat, args=(s, stop, deadman), daemon=True).start()
    return s, stop, deadman


def test_demo_calibration():
    cfg = json.loads(json.dumps(CFG))
    cfg['calibration']['seek_speed_dps'] = 40  # quicker than on hardware; the demo motors keep up
    with tempfile.TemporaryDirectory() as tmp:
        cal_dir = Path(tmp)
        s, stop, deadman = run_session(cfg, cal_dir)
        try:
            s.request('connect', demo=True)
            wait(s, lambda d: d['state'].startswith('Timeout'), 3, 'timeout latch')
            s.request('ack')
            d = wait(s, lambda d: d['state'] == 'Ready · Stop', 3, 'ready')
            assert not d['zeroed'] and 'Not calibrated' in d['reason']
            s.request('arm', supported=True)
            wait(s, lambda d: 'Calibrate before arming' in d['reason'], 2, 'arm refused')

            s.request('calibrate', supported=True)
            d = wait(s, lambda d: d['calibration']['awaiting_confirm'], 2, 'J1 confirm prompt')
            assert d['state'] == 'Calibrating' and not d['armed']
            s.request('cal_confirm')
            d = wait(s, lambda d: d['calibration']['steps'][1]['status'] == 'running', 3, 'J2 running')
            assert not d['armed'], 'calibration FOC is not "armed": the sim must snap when arming completes'
            # Deadman released: nothing moves.
            p0 = [f['position_deg'] for f in s.snapshot()['feedback']]
            time.sleep(.6)
            p1 = [f['position_deg'] for f in s.snapshot()['feedback']]
            assert max(abs(a - b) for a, b in zip(p0, p1)) < .2, (p0, p1)
            deadman.set()
            t0 = time.monotonic()
            d = wait(s, lambda d: d['state'].startswith('Armed') or 'failed' in d['state'], 90, 'calibration done')
            # Still holding RB when it finishes: hold, don't follow, until RB is released and pressed again.
            time.sleep(.5)
            d = s.snapshot()
            assert d['state'] == 'Armed · holding', (d['state'], d['reason'], d['calibration'])
            steps = d['calibration']['steps']
            print(f'  demo calibration finished in {time.monotonic() - t0:.1f}s (deadman held)')
            for st in steps:
                print(f"   {CFG['joints'][st['joint']]['name']:12s} {st['method']:10s} {st['status']:5s} "
                      + ' '.join(f'{k}={st[k]}' for k in ('measured_span_deg', 'span_error_deg', 'q_ref_deg', 'limits_deg') if k in st))
                assert st['status'] == 'done'
            for i in range(3):
                q = d['feedback'][i]['q_deg']
                assert abs(q - truth_deg(s, i)) < 0.6, (i, q, truth_deg(s, i))
            assert d['limits_deg'][1][0] > -7.59 and d['limits_deg'][1][1] < 53.11, d['limits_deg']
            saved = json.loads((cal_dir / 'demo.json').read_text())
            assert [j['method'] for j in saved['joints']] == ['manual', 'hard_stops', 'contacts']
            s.request('stop')
            wait(s, lambda d: d['state'].startswith('Stopped'), 2, 'stop')
        finally:
            stop.set()
            s.close()

        # A new session (power cycle): calibrated straight from the file, no homing.
        s, stop, deadman = run_session(cfg, cal_dir)
        try:
            s.request('connect', demo=True)
            wait(s, lambda d: d['state'].startswith('Timeout'), 3, 'timeout latch')
            s.request('ack')
            d = wait(s, lambda d: d['state'] == 'Ready · Stop', 3, 'ready')
            assert d['zeroed'] and d['calibration']['saved'], d['calibration']
            for i in range(3):
                assert abs(d['feedback'][i]['q_deg'] - truth_deg(s, i)) < 0.6
            s.request('arm', supported=True)
            wait(s, lambda d: d['armed'], 3, 'armed from saved calibration')
        finally:
            stop.set()
            s.close()
    print('demo calibration: ok (recovers hidden mounting angles; reconnect needs no homing)')


def test_span_mismatch_fails_safe():
    cfg = json.loads(json.dumps(CFG))
    cfg['calibration']['seek_speed_dps'] = 40
    manifest = json.loads(json.dumps(MANIFEST))
    j2 = next(st for st in manifest['calibration']['steps'] if st['joint'] == 1)
    real = dict(j2['predicted'])
    with tempfile.TemporaryDirectory() as tmp:
        # The demo motors keep the real stops; the "CAD" now claims a 20 deg wider range.
        j2['predicted'] = json.loads(json.dumps(real))
        j2['predicted']['hi']['deg'] += 20
        s = ArmSession(cfg, codec, sim, manifest, dict(real=Path(tmp) / 'r.json', demo=Path(tmp) / 'd.json'))
        stop, deadman = threading.Event(), threading.Event()
        deadman.set()
        threading.Thread(target=heartbeat, args=(s, stop, deadman), daemon=True).start()
        try:
            # demo_buses reads stops from the plan, so give it the real ones.
            s.plan = json.loads(json.dumps(MANIFEST['calibration']))
            s.request('connect', demo=True)
            wait(s, lambda d: d['state'].startswith('Timeout'), 3, 'timeout')
            s.plan = manifest['calibration']
            s.request('ack')
            wait(s, lambda d: d['state'] == 'Ready · Stop', 3, 'ready')
            s.request('calibrate', supported=True)
            wait(s, lambda d: d['calibration']['awaiting_confirm'], 2, 'confirm')
            s.request('cal_confirm')
            d = wait(s, lambda d: 'failed' in d['state'], 60, 'span failure')
            assert 'span' in d['reason'] and not d['armed'], d['reason']
            assert all(f['mode'] == 0 for f in d['feedback'])
            assert not (Path(tmp) / 'd.json').exists(), 'a failed calibration must not be saved'
        finally:
            stop.set()
            s.close()
    print('span mismatch: ok (stops, disarms, saves nothing)')


if __name__ == '__main__':
    test_math()
    test_demo_calibration()
    test_span_mismatch_fails_safe()
    print('ALL CALIBRATION TESTS PASSED')
