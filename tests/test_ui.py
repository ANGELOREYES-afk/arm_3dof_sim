"""End-to-end UI check: headless Chromium + a stubbed Xbox pad + the bridge with demo motors.

    pip install playwright && playwright install chromium   # once, optional dependency
    python3 tests/test_ui.py                               # writes tests/out/*.png

Runs the real page against the real bridge. The demo motors are im6014Development's packet
simulator with absolute encoders and CAD-predicted stops (bridge/demo_motors.py). The bridge gets
a temporary config (faster calibration sweeps) so nothing is written to config/.
"""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

try:
    from playwright.sync_api import sync_playwright
except ImportError:
    sys.exit('SKIP: playwright not installed (pip install playwright && playwright install chromium)')

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'tests' / 'out'
PORT = 8797

PAD_STUB = """
window.__pad = { id: 'Xbox Wireless Controller (STANDARD GAMEPAD Vendor: 045e Product: 0b13)', mapping: 'standard',
  connected: true, axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => ({ pressed: false, value: 0 })) };
Object.defineProperty(navigator, 'getGamepads', { value: () => [window.__pad], configurable: true });
window.__press = (i, on) => { window.__pad.buttons[i] = { pressed: on, value: on ? 1 : 0 }; };
"""
A, B, X, Y, LB, RB, MENU = 0, 1, 2, 3, 4, 5, 9


def main():
    OUT.mkdir(exist_ok=True)
    tmp = tempfile.TemporaryDirectory()
    cfg = json.loads((ROOT / 'config' / 'arm.json').read_text())
    cfg['calibration']['seek_speed_dps'] = 40
    (Path(tmp.name) / 'arm.json').write_text(json.dumps(cfg))
    python = os.environ.get('BRIDGE_PYTHON', sys.executable)  # needs pyserial
    bridge = subprocess.Popen([python, str(ROOT / 'bridge' / 'arm_bridge.py'), '--port', str(PORT),
                               '--config', str(Path(tmp.name) / 'arm.json')],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        for _ in range(50):
            try:
                socket.create_connection(('127.0.0.1', PORT), timeout=.2).close()
                break
            except OSError:
                time.sleep(.1)
        with sync_playwright() as pw:
            # Hardware GL: software GL blocks the main thread long enough to (correctly) trip the heartbeat.
            browser = pw.chromium.launch(args=['--use-angle=metal', '--ignore-gpu-blocklist', '--enable-gpu'])
            page = browser.new_page(viewport=dict(width=1440, height=900))
            errors = []
            page.on('pageerror', lambda e: errors.append(str(e)))
            page.on('console', lambda m: m.type == 'error' and errors.append(m.text))
            page.add_init_script(PAD_STUB)
            page.goto(f'http://127.0.0.1:{PORT}/')
            page.wait_for_selector('#loading', state='detached', timeout=60000)
            fps = page.evaluate("""() => new Promise(r => { let n = 0; const t0 = performance.now();
              (function f() { n++; performance.now() - t0 < 1000 ? requestAnimationFrame(f) : r(n); })(); })""")
            print(f'render: {fps} fps (headless)')
            if fps < 4:
                sys.exit(f'FAIL: {fps} fps is too slow to keep the 0.5 s bridge heartbeat alive')
            assert page.text_content('#pad-name') == 'Xbox Wireless Controller'
            page.screenshot(path=OUT / '1_sim.png')

            state = lambda: page.text_content('#m-state')
            press = lambda b: (page.evaluate(f'__press({b}, true)'), time.sleep(.15), page.evaluate(f'__press({b}, false)'))

            def wait_state(pred, what, timeout=8):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    if pred(state()):
                        return
                    time.sleep(.05)
                log = page.eval_on_selector_all('#m-events li', 'els => els.slice(-8).map(e => e.textContent)')
                raise AssertionError(f'waiting for {what}; state={state()!r} reason={page.text_content("#m-reason")!r} '
                                     f'cal={page.text_content("#cal-msg")!r} log={log}')

            q_meas = lambda: [float(x) if x != '—' else None for x in page.eval_on_selector_all('#m-table tr td:nth-child(3)', 'els => els.map(e => e.textContent)')]
            q_sim = lambda: [float(t.rstrip('°')) for t in page.eval_on_selector_all('.joint .val', 'els => els.map(e => e.textContent)')]

            page.click('#m-connect')
            wait_state(lambda s: s.startswith('Timeout'), 'timeout latch')
            page.click('#m-ack')
            wait_state(lambda s: s == 'Ready · Stop', 'ready')
            assert 'Not calibrated' in page.text_content('#m-reason'), page.text_content('#m-reason')
            page.check('#m-supported')
            time.sleep(.3)  # the panel refreshes at 10 Hz
            assert page.is_disabled('#m-arm') and not page.is_disabled('#cal-start')

            # Calibrate: J1 manual (confirm with the Menu button), then J2/J3 seek while RB is held.
            page.click('#cal-start')
            wait_state(lambda s: s == 'Calibrating', 'calibrating')
            page.wait_for_selector('#cal-confirm:not([hidden])', timeout=5000)
            press(MENU)
            page.wait_for_selector('#cal-confirm', state='hidden', timeout=5000)
            time.sleep(.5)
            assert 'hold RB' in page.text_content('#hud-arm'), page.text_content('#hud-arm')
            page.evaluate(f'__press({RB}, true)')
            time.sleep(2)
            page.screenshot(path=OUT / '2_calibrating.png')
            t0 = time.monotonic()
            wait_state(lambda s: s.startswith('Armed') or 'failed' in s, 'calibration done', timeout=90)
            assert state() == 'Armed · holding', (state(), page.text_content('#m-reason'))
            print(f'  calibrated in the browser in {time.monotonic() - t0 + 2:.0f}s')
            steps = page.eval_on_selector_all('#cal-steps li', 'els => els.map(e => e.className + ": " + e.innerText.replace(/\\n/g, " "))')
            for s in steps:
                print('   ', s[:170])
                assert s.startswith('done'), s
            assert 'Saved' in page.text_content('#cal-saved')
            page.screenshot(path=OUT / '3_calibrated.png')

            # Still holding RB from calibration: the arm holds until RB is released and pressed again.
            time.sleep(.5)
            assert 'holding' in page.text_content('#hud-arm')
            page.evaluate(f'__press({RB}, false)')
            press(X)  # sim to the ready pose; motors hold (deadman released)
            end = time.monotonic() + 15
            while time.monotonic() < end and abs(q_sim()[2] - cfg['ready_pose_deg'][2]) > .5:
                time.sleep(.1)
            page.evaluate(f'__press({RB}, true)')
            time.sleep(.5)
            assert 'FOLLOWING' in page.text_content('#hud-arm'), page.text_content('#hud-arm')
            end = time.monotonic() + 15
            while time.monotonic() < end and max(abs(a - b) for a, b in zip(q_meas(), cfg['ready_pose_deg'])) > 1.5:
                time.sleep(.1)
            m = q_meas()
            print(f'  motors at the ready pose: {m} (target {cfg["ready_pose_deg"]})')
            assert max(abs(a - b) for a, b in zip(m, cfg['ready_pose_deg'])) < 1.5, m
            page.evaluate('__pad.axes[1] = -1')  # LS up while driving
            time.sleep(1.2)
            page.evaluate('__pad.axes[1] = 0')
            time.sleep(1.2)
            sim, m = q_sim(), q_meas()
            print(f'  after LS with RB held: sim {sim} motors {m}')
            assert max(abs(a - b) for a, b in zip(sim, m)) < 1.5, (sim, m)
            page.screenshot(path=OUT / '4_driving.png')

            press(B)
            wait_state(lambda s: s.startswith('Stopped'), 'stopped')
            page.evaluate(f'__press({RB}, false)')

            # Power-cycle stand-in: disconnect and reconnect. Absolute encoders + the saved file
            # mean it is calibrated straight away.
            page.click('#m-disconnect')
            wait_state(lambda s: s == 'Disconnected', 'disconnected')
            page.click('#m-connect')
            wait_state(lambda s: s.startswith('Timeout'), 'timeout latch')
            page.click('#m-ack')
            wait_state(lambda s: s == 'Ready · Stop', 'ready')
            assert None not in q_meas(), 'should be calibrated from the saved file'
            time.sleep(.3)
            # ...but a loaded calibration is verified once per connection before arming.
            assert page.is_disabled('#m-arm') and page.is_visible('#cal-verify')
            page.click('#cal-verify')
            wait_state(lambda s: s == 'Verifying', 'verifying')
            page.evaluate(f'__press({RB}, true)')
            page.wait_for_selector('#cal-confirm:not([hidden])', timeout=30000)
            page.evaluate(f'__press({RB}, false)')
            page.screenshot(path=OUT / '5_verify.png')
            press(MENU)  # the ghost lies on the (simulated) arm: confirm
            wait_state(lambda s: s.startswith('Armed'), 'armed after verify')
            assert 'verified this power-up' in page.text_content('#cal-saved')
            print('  verify after reconnect: ' + ' | '.join(page.eval_on_selector_all('#cal-steps li', 'els => els.map(e => e.innerText.replace(/\\n/g, " ").slice(0, 90))')))
            assert page.text_content('#m-lib').startswith('Transport: im6014'), page.text_content('#m-lib')

            # Sim-only calibration preview runs through the plan.
            press(B)
            wait_state(lambda s: s.startswith('Stopped'), 'stopped')
            page.click('#cal-preview')
            time.sleep(3)
            assert 'Sim →' in page.text_content('#hud-mode')
            page.screenshot(path=OUT / '6_preview.png')
            assert not errors, errors
            browser.close()
    finally:
        bridge.terminate()
        out = bridge.communicate(timeout=5)[0].decode()
        if 'Traceback' in out:
            print(out)
            sys.exit('FAIL: bridge traceback')
        tmp.cleanup()
    print('ui: ok — screenshots in tests/out/')


if __name__ == '__main__':
    main()
