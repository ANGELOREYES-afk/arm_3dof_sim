"""End-to-end UI check: headless Chromium + a stubbed Xbox pad + the bridge in demo mode.

    pip install playwright && playwright install chromium   # once, optional dependency
    python3 tests/test_ui.py                               # writes tests/out/*.png

Drives the real page and the real bridge; motors are im6014Development's packet simulator.
"""
import os
from pathlib import Path
import socket
import subprocess
import sys
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
A, B, X, Y, LB, RB = 0, 1, 2, 3, 4, 5


def main():
    OUT.mkdir(exist_ok=True)
    python = os.environ.get('BRIDGE_PYTHON', sys.executable)  # needs pyserial
    bridge = subprocess.Popen([python, str(ROOT / 'bridge' / 'arm_bridge.py'), '--port', str(PORT)],
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
            page.wait_for_selector('#loading', state='detached', timeout=30000)
            fps = page.evaluate("""() => new Promise(r => { let n = 0; const t0 = performance.now();
              (function f() { n++; performance.now() - t0 < 1000 ? requestAnimationFrame(f) : r(n); })(); })""")
            print(f'render: {fps} fps (headless)')
            if fps < 4:
                sys.exit(f'FAIL: {fps} fps is too slow to keep the 0.5 s bridge heartbeat alive')
            assert page.text_content('#pad-name') == 'Xbox Wireless Controller'
            page.screenshot(path=OUT / '1_sim.png')

            state = lambda: page.text_content('#m-state')
            def wait_state(pred, what, timeout=6):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    if pred(state()):
                        return
                    time.sleep(.05)
                raise AssertionError(f'waiting for {what}; state={state()!r} reason={page.text_content("#m-reason")!r}')
            meas = lambda: [float(x) for x in page.eval_on_selector_all('#m-table tr td:nth-child(3)', 'els => els.map(e => e.textContent)')]

            page.click('#m-connect')
            wait_state(lambda s: s.startswith('Timeout'), 'timeout latch')
            assert page.is_disabled('#m-arm')
            page.click('#m-ack')
            wait_state(lambda s: s == 'Ready · Stop', 'ready')
            page.check('#m-supported')
            page.click('#m-arm')
            wait_state(lambda s: s.startswith('Armed'), 'armed')
            assert max(map(abs, meas())) < 1, meas()

            # X without the deadman: the sim moves to the ready pose, the motors hold.
            page.evaluate(f'__press({X}, true)')
            time.sleep(.15)
            page.evaluate(f'__press({X}, false)')
            time.sleep(2.5)
            assert max(map(abs, meas())) < 1, f'motors moved without the deadman: {meas()}'
            assert 'holding' in page.text_content('#hud-arm'), (page.text_content('#hud-arm'), state(), page.text_content('#m-reason'))

            # Hold RB: motors catch up to the sim at their configured speed limits.
            sim_q = lambda: [float(t.rstrip('°')) for t in page.eval_on_selector_all('.joint .val', 'els => els.map(e => e.textContent)')]
            end = time.monotonic() + 10
            while time.monotonic() < end and abs(sim_q()[2] - 65) > .5:
                time.sleep(.1)
            assert abs(sim_q()[1] - 25) < .5 and abs(sim_q()[2] - 65) < .5, sim_q()
            page.evaluate(f'__press({RB}, true)')
            time.sleep(.6)
            assert 'FOLLOWING' in page.text_content('#hud-arm'), page.text_content('#hud-arm')
            time.sleep(1.8)
            m = meas()
            print(f'  motors after holding RB: {m} (sim at ready pose 0 / 25 / 65)')
            assert abs(m[1] - 25) < 2 and abs(m[2] - 65) < 2, m
            # Drive with the left stick while holding RB; the motors track the IK solution.
            page.evaluate('__pad.axes[0] = 1')
            time.sleep(1.5)
            page.evaluate('__pad.axes[0] = 0')
            time.sleep(1.0)
            m2 = meas()
            sim = sim_q()
            print(f'  after LS → with RB held: sim {sim} motors {m2}')
            assert abs(m2[0] - sim[0]) < 2 and abs(m2[0]) > 3, (sim, m2)
            page.screenshot(path=OUT / '2_driving.png')

            # Release RB: the motors hold while the sim previews ahead.
            page.evaluate(f'__press({RB}, false)')
            page.evaluate('__pad.axes[1] = 1')
            time.sleep(1.0)
            page.evaluate('__pad.axes[1] = 0')
            time.sleep(.5)
            assert max(abs(a - b) for a, b in zip(meas(), m2)) < 1, 'motors moved after deadman release'

            # B: Stop and disarm.
            page.evaluate(f'__press({B}, true)')
            time.sleep(.2)
            page.evaluate(f'__press({B}, false)')
            wait_state(lambda s: s.startswith('Stopped'), 'stopped')
            assert 'Torque released' in page.text_content('#m-reason')
            page.screenshot(path=OUT / '3_stopped.png')

            # Figure-8 demo (sim only) traces a path.
            page.evaluate(f'__press({LB}, true)')
            time.sleep(.15)
            page.evaluate(f'__press({LB}, false)')
            time.sleep(4)
            assert 'Figure-8' in page.text_content('#hud-mode')
            page.screenshot(path=OUT / '4_figure8.png')

            # Joint mode: A toggles; LS drives J1 directly.
            page.evaluate(f'__press({LB}, true)'); time.sleep(.15); page.evaluate(f'__press({LB}, false)')
            page.evaluate(f'__press({A}, true)'); time.sleep(.15); page.evaluate(f'__press({A}, false)')
            time.sleep(.3)
            assert page.text_content('#hud-mode') == 'Joint mode'
            assert not errors, errors
            browser.close()
    finally:
        bridge.terminate()
        out = bridge.communicate(timeout=5)[0].decode()
        if 'Traceback' in out:
            print(out)
            sys.exit('FAIL: bridge traceback')
    print('ui: ok — screenshots in tests/out/')


if __name__ == '__main__':
    main()
