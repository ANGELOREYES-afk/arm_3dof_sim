"""Three-joint IM6014 session: one serial-owner thread streaming joint-space position targets.

The packet codec, health gate and packet-level simulator come unchanged from the IM6014 repo
(bridge/im6014.py). With sidrk-dev/im6014-workbench this also uses its transaction (resyncs past
corrupt or foreign-ID frames on the shared bus, enforces the deadline, typed errors), its packet
capture for diagnostics, its JointMap algebra, and its per-device port lease on Linux.

This module adds the arm: several IDs on one bus, absolute-encoder joint calibration and
power-up verification (calibration.py), joint limits, an acceleration-limited trajectory toward
the browser's target, a predicted-effort guard, and an operator deadman. It keeps the bench
conventions: starts disconnected and disarmed, Stop/status only until armed, every FOC packet
sets the firmware watchdog, any fault or stale heartbeat/feedback latches Stop and disarm, a fault
clears only by explicit acknowledgment plus a fresh Stop check, and nothing resumes on its own.

Stop releases torque. A gravity-loaded arm will fall; support it.
"""
from collections import deque
import copy
import datetime
import json
import math
import queue
import sys
import threading
import time

from calibration import ContactSeek, SeekSettings, joint_angle, solve_two_contacts
import demo_motors

TICK_S = 0.01
IDLE_POLL_S = 0.05


class ArmSession:
    def __init__(self, cfg, lib, manifest, cal_paths):
        self.cfg = cfg
        self.lib = lib                       # bridge/im6014.Im6014
        self.codec, self.sim = lib.codec, lib.sim
        self.plan = manifest['calibration']  # from tools/import_urdf.py
        self.analytic = manifest['analytic']
        self.cal_paths = cal_paths           # {'real': Path, 'demo': Path}
        self.joints = cfg['joints']
        self.safety = cfg['safety']
        self.n = len(self.joints)
        self.cfg_lim = [(math.radians(j['min_deg']), math.radians(j['max_deg'])) for j in self.joints]
        self.lim = list(self.cfg_lim)
        self.lock = threading.RLock()
        self.jobs = queue.Queue()
        self.stop_requested = threading.Event()
        self.closing = threading.Event()
        self.generation = 0
        self.buses = []          # per joint; joints on the same port share one serial object
        self.leases = []         # workbench ManagedPorts to close (Linux)
        self.capture = lib.serial.PacketCapture() if lib.serial else None
        self.connection = 0
        self.refs = [None] * self.n   # calibration per joint (see calibration.joint_angle)
        self.zero = [None] * self.n   # this session: motor output angle (rad) where q = 0
        self.maps = [None] * self.n   # workbench JointMap per joint, once zero is known
        self.q_cmd = None
        self.v_cmd = None
        self.cal = None          # running calibration or verification
        self.target = None       # latest browser target (rad), its time and deadman state
        self.target_at = 0.
        self.deadman = False
        self.need_release = False  # after arming, the deadman must be released once before following
        self.last_ui = 0.
        self.last_reply = [0.] * self.n
        self.last_poll = 0.
        self.last_tick = time.monotonic()
        self.started = time.monotonic()
        self.events = deque(maxlen=300)
        self.event_seq = 0
        self.data = dict(state='Disconnected', reason='Connect to the motors (demo or real).', connected=False,
                         demo=False, armed=False, moving=False, zeroed=False, verified=False, effort_limited=None,
                         port=cfg['bus']['port'], baud=cfg['bus']['baud'], library=lib.name,
                         tx=0, rx=0, crc_errors=0, rejected_frames=0, reply_timeouts=0, loop_hz=0., cycle_ms=0.,
                         feedback=[None] * self.n, q_cmd=None, limits_deg=self._limits_deg(),
                         calibration=dict(active=False, mode=None, saved=None, steps=[], message='', awaiting_confirm=False))
        self.thread = threading.Thread(target=self._loop, daemon=True, name='arm serial owner')
        self.thread.start()

    # ---- thread-safe API (called from the web server threads) ----
    def ping(self):
        self.last_ui = time.monotonic()

    def set_target(self, q, deadman):
        if len(q) != self.n or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in q):
            raise ValueError('Target must be finite joint angles in radians')
        with self.lock:
            self.target = [float(v) for v in q]
            self.target_at = time.monotonic()
            self.deadman = bool(deadman)
            self.last_ui = self.target_at

    def request(self, action, **values):
        if action == 'stop':
            with self.lock:
                self.generation += 1
            self.stop_requested.set()
            return
        self.jobs.put((self.generation, action, values))

    def snapshot(self, since=0):
        now = time.monotonic()
        with self.lock:
            d = copy.deepcopy(self.data)
            d['events'] = [e for e in self.events if e[0] > since]
        for f in d['feedback']:
            if f:
                f['age_ms'] = round((now - f.pop('at')) * 1000)
        return d

    def diagnostics(self):
        """Everything worth attaching to a bug report: state, events, recent packets."""
        snap = self.snapshot()
        snap['events'] = [e[1] for e in snap['events']]
        return dict(created=datetime.datetime.now().isoformat(timespec='seconds'), library=self.lib.name,
                    library_path=str(self.lib.path), config=self.cfg, state=snap,
                    packets=self.capture.snapshot() if self.capture else None)

    def close(self):
        self.request('stop')
        self.closing.set()
        self.thread.join(2)

    # ---- internals (serial-owner thread only) ----
    def _event(self, text):
        with self.lock:
            self.event_seq += 1
            self.events.append((self.event_seq, f'{time.monotonic() - self.started:8.2f}s  {text}'))

    def _set(self, **values):
        with self.lock:
            self.data.update(values)

    def _set_cal(self, **values):
        with self.lock:
            self.data['calibration'].update(values)

    def _count(self, **inc):
        with self.lock:
            for k, v in inc.items():
                self.data[k] += v

    def _centre_deg(self, i):
        """Middle of joint i's configured limits: the absolute-encoder mapping is unambiguous
        within +-180 deg of it."""
        return math.degrees(sum(self.cfg_lim[i])) / 2

    def _limits_deg(self):
        return [[round(math.degrees(a), 2), round(math.degrees(b), 2)] for a, b in self.lim]

    def _set_zero(self, zero):
        """This session's joint <-> motor mapping (the workbench JointMap when available)."""
        self.zero = zero
        jm = self.lib.mapping
        self.maps = [jm.JointMap(1.0, j['direction'], z) if jm and z is not None else None
                     for j, z in zip(self.joints, zero)]

    def _q(self, i, f):
        if self.zero[i] is None:
            return None
        if self.maps[i]:
            return self.maps[i].from_output(self.lib.mapping.OutputSample(f['position_rad'], f['rpm'], f['torque_nm'])).q_rad
        return self.joints[i]['direction'] * (f['position_rad'] - self.zero[i])

    def _transact(self, i, mode=0, watchdog=True, enc=0., rpm=0., torque=0., kp=None, kd=None):
        """One exchange with joint i. FOC targets are in the motor's output frame (rad, rpm, Nm)."""
        if mode and (self.stop_requested.is_set() or self.closing.is_set()):
            raise InterruptedError('Stop requested')
        j = self.joints[i]
        if mode:
            packet = self.codec.command(j['id'], 1, rpm=rpm, kd=j['kd'] if kd is None else kd, torque=torque,
                                        position=enc, kp=j['kp'] if kp is None else kp)
        else:
            packet = self.codec.command(j['id'], 0, watchdog=watchdog)
        self._count(tx=1)
        started = time.monotonic()
        try:
            if self.capture:
                demo = self.data['demo']
                port = j.get('port', self.data['port'])
                f = self.lib.serial.capture_exchange(
                    self.capture, self.codec.exchange, self.buses[i], packet, j['id'], source='demo' if demo else 'physical',
                    identity='demo motors' if demo else port, connection=self.connection, port='' if demo else port,
                    baud=self.data['baud'])
            else:
                f = self.codec.exchange(self.buses[i], packet, j['id'])
        except Exception as error:
            typed = self.lib.serial and isinstance(error, self.lib.serial.TransactionTimeoutError)
            if typed:
                self._count(reply_timeouts=1, rejected_frames=error.rejected_frames, crc_errors=error.crc_rejected_frames)
            else:  # im6014Development: classify by message
                self._count(crc_errors=int('CRC' in str(error)), reply_timeouts=int('deadline' in str(error)))
            raise RuntimeError(f"{j['name']} (ID {j['id']}): {error}") from None
        now = time.monotonic()
        self.last_reply[i] = now
        q_meas = self._q(i, f)
        fb = dict(mode=f['mode'], timeout=f['timeout'], position_deg=math.degrees(f['position_rad']),
                  q_deg=None if q_meas is None else math.degrees(q_meas), rpm=f['rpm'], torque_nm=f['torque_nm'],
                  voltage=f['voltage'], driver_c=f['driver_c'], winding_c=f['winding_c'],
                  faults=self.sim.fault_text(f['errors'], f['warnings']), rtt_ms=round((now - started) * 1000, 2), at=now)
        with self.lock:
            self.data['rx'] += 1
            self.data['feedback'][i] = fb
        return f

    def _foc_q(self, i, q, vel=0., tau=0.):
        """FOC toward joint angle q (rad) with joint velocity/torque feed-forward."""
        j = self.joints[i]
        if self.maps[i]:
            o = self.maps[i].to_output(self.lib.mapping.JointTarget(q, vel, tau, j['kp'], j['kd']))
            return self._transact(i, 1, enc=o.q_rad, rpm=o.rpm, torque=o.tau_nm, kp=o.kp, kd=o.kd)
        s = j['direction']
        return self._transact(i, 1, enc=self.zero[i] + s * q, rpm=s * vel * 60 / math.tau, torque=s * tau)

    def _gate(self, i, f, mode, allow_timeout=False):
        try:
            self.codec.healthy(f, mode, allow_timeout=allow_timeout)
        except RuntimeError as error:
            raise RuntimeError(f"{self.joints[i]['name']}: {error}") from None

    def _stop_all(self):
        """Stop every joint; True only if each confirmed mode 0."""
        self.q_cmd = self.v_cmd = None
        self.cal = None
        self._set(armed=False, moving=False, q_cmd=None, effort_limited=None)
        self._set_cal(active=False, awaiting_confirm=False)
        if not self.buses:
            return False
        ok = True
        for i in range(self.n):
            try:
                f = self._transact(i)
                if f['mode'] != 0:
                    f = self._transact(i)
                ok &= f['mode'] == 0
            except Exception as error:
                ok = False
                self._event(f'STOP NOT CONFIRMED on {self.joints[i]["name"]}: {error}')
        return ok

    def _fault(self, message):
        was_cal = self.cal is not None
        self._stop_all()
        self._set(state='Fault — disarmed', reason=message + ' Torque released — support the arm, fix the cause, then Acknowledge.')
        if was_cal:
            self._set_cal(message='Aborted: ' + message)
        self._event('DISARMED: ' + message)

    def _open(self, demo, port, baud):
        if demo:  # one shared bus, like the real wiring
            bus = demo_motors.demo_bus(self.sim, self.codec, self.joints, self.plan)
            return [bus] * self.n
        buses, shared = [], {}
        try:
            for j in self.joints:
                p = j.get('port', port)
                if p not in shared:
                    shared[p] = self._open_port(p, baud)
                buses.append(shared[p])
        except Exception:
            opened = list(shared.values())
            self._close_buses(opened)
            self.leases = [lease for lease in self.leases if all(lease.bus is not b for b in opened)]
            raise
        return buses

    def _open_port(self, path, baud):
        b = self.cfg['bus']
        if self.lib.port and sys.platform.startswith('linux'):
            # Workbench lease: one owner per device, verified after open. Not on macOS, where /tmp
            # is a symlink and the lease refuses to run.
            route = self.lib.port.PortConfig(path, baud, b.get('expected_serial'), b.get('expected_channel'))
            lease = self.lib.port.open_managed_port(route)
            self.leases.append(lease)
            self._event(f'Port lease held on {path}.')
            return lease.bus
        import serial  # same open sequence as motor_session.MotorSession
        s = serial.Serial(port=None, baudrate=baud, timeout=0.005, write_timeout=0.05, exclusive=True)
        s.port = path
        s.dtr = s.rts = False
        s.open()
        return s

    def _close_buses(self, buses=None):
        leased = {id(lease.bus): lease for lease in self.leases}
        for bus in {id(b): b for b in (self.buses if buses is None else buses)}.values():
            try:
                if id(bus) in leased:
                    if not leased[id(bus)].close():
                        self._event('Port closed, but its lease could not be released.')
                else:
                    bus.close()
            except Exception as error:
                self._event(f'Closing the port failed: {error}')
        if buses is None:
            self.buses, self.leases = [], []

    # ---- calibration file and mapping ----
    def _cal_path(self):
        return self.cal_paths['demo' if self.data['demo'] else 'real']

    def _load_refs(self):
        path = self._cal_path()
        self.refs = [None] * self.n
        self.lim = list(self.cfg_lim)
        self._set(limits_deg=self._limits_deg())
        if not path.is_file():
            self._set_cal(saved=None)
            return
        try:
            saved = json.loads(path.read_text())
            for i, (j, r) in enumerate(zip(self.joints, saved['joints'])):
                if r['id'] != j['id'] or r['direction'] != j['direction']:
                    raise ValueError(f"{j['name']}: saved ID/direction differ from config/arm.json")
                self.refs[i] = r
            self._apply_limits()
            self._set_cal(saved=dict(created=saved['created'], methods=[r['method'] for r in saved['joints']],
                                     file=path.name))
            self._event(f'Loaded calibration {path.name} ({saved["created"]}).')
        except Exception as error:
            self.refs = [None] * self.n
            self._set_cal(saved=None)
            self._event(f'Ignoring {path.name}: {error}')

    def _apply_limits(self):
        lim = []
        for (lo, hi), r in zip(self.cfg_lim, self.refs):
            if r and r.get('limits_deg'):
                lo, hi = max(lo, math.radians(r['limits_deg'][0])), min(hi, math.radians(r['limits_deg'][1]))
            lim.append((lo, hi))
        self.lim = lim
        self._set(limits_deg=self._limits_deg())

    def _apply_refs(self):
        """Absolute encoders: map each joint's current reading to q and fix this session's zero."""
        fb = self.data['feedback']
        zero = [None] * self.n
        for i, r in enumerate(self.refs):
            if r and fb[i]:
                q = math.radians(joint_angle(r, fb[i]['position_deg'], self._centre_deg(i)))
                zero[i] = math.radians(fb[i]['position_deg']) - r['direction'] * q
                with self.lock:
                    fb[i]['q_deg'] = math.degrees(q)
        self._set_zero(zero)
        zeroed = all(z is not None for z in zero)
        self._set(zeroed=zeroed)
        return zeroed

    def _save_refs(self, refs, source):
        doc = dict(version=1, created=datetime.datetime.now().isoformat(timespec='seconds'), source=source,
                   note='Absolute output-encoder calibration: q = q_ref + direction * wrap(enc - enc_ref), degrees, '
                        'wrapped to +-180 of the middle of the joint limits.',
                   joints=[{**r, 'name': j['name'], 'id': j['id'], 'direction': j['direction']} for j, r in zip(self.joints, refs)])
        path = self._cal_path()
        path.write_text(json.dumps(doc, indent=2) + '\n')
        self._event(f'Saved calibration to {path.name}.')
        self._load_refs()

    def _check(self, acknowledge=False):
        if not self.buses:
            raise ValueError('Connect first')
        if self.data['state'].startswith(('Fault', 'Calibration failed', 'Verification failed')) and not acknowledge:
            raise ValueError('Acknowledge the fault first: fix its cause, support the arm, then Acknowledge')
        if not self._stop_all():
            raise RuntimeError('Cannot verify Stop on every joint; check adapter, IDs, baud and motor power')
        timed_out = []
        for i in range(self.n):
            f = self._transact(i)
            self._gate(i, f, 0, allow_timeout=True)
            if f['timeout']:
                if not acknowledge:
                    timed_out.append(self.joints[i]['name'])
                    continue
                self._transact(i, watchdog=False)
                f = self._transact(i)
                self._gate(i, f, 0)
                self._event(f'{self.joints[i]["name"]}: startup timeout acknowledged in Stop; watchdog restored.')
        if timed_out:
            self._set(state='Timeout — disarmed', reason='Timeout latched on ' + ', '.join(timed_out) +
                      '. Support the arm, then Acknowledge.')
            return
        if acknowledge:
            self._event('Acknowledged; Stop confirmed with healthy feedback on every joint.')
        reason = 'Healthy feedback, all joints stopped.'
        if self._apply_refs():
            outside = [j['name'] for i, j in enumerate(self.joints)
                       if not self.lim[i][0] - math.radians(5) <= math.radians(self.data['feedback'][i]['q_deg']) <= self.lim[i][1] + math.radians(5)]
            if outside:
                reason += (f" {', '.join(outside)} reads outside its calibrated range — move it back by hand, "
                           'or recalibrate if the joint was reassembled.')
            if self._needs_verify():
                reason += ' Calibration loaded from file: Verify it once after power-up before arming.'
        else:
            reason += ' Not calibrated: run Calibrate (or Set reference at the calibration pose).'
        self._set(state='Ready · Stop', reason=reason)

    def _needs_verify(self):
        return bool(self.safety.get('verify_after_connect', True) and self.data['zeroed'] and not self.data['verified'])

    def _measured(self):
        return [math.radians(f['q_deg']) for f in self.data['feedback']]

    # ---- requests ----
    def _handle(self, action, v):
        if action == 'connect':
            if self.buses:
                raise ValueError('Disconnect first')
            demo = bool(v.get('demo'))
            port = v.get('port') or self.cfg['bus']['port']
            baud = v.get('baud', self.cfg['bus']['baud'])
            if baud not in (4000000, 6000000):
                raise ValueError('Baud must be 4 or 6 Mbps')
            ids = [j['id'] for j in self.joints]
            if len(set(ids)) != len(ids) or not all(0 <= x <= 14 for x in ids):
                raise ValueError('Joint motor IDs must be unique and 0–14')
            self._set_zero([None] * self.n)
            self.connection += 1
            self._set(state='Connecting', reason='Opening adapter, sending Stop only.', demo=demo, port=port,
                      baud=baud, zeroed=False, verified=False, feedback=[None] * self.n, tx=0, rx=0, crc_errors=0,
                      rejected_frames=0, reply_timeouts=0)
            self.buses = self._open(demo, port, baud)
            self._set(connected=True)
            self._event(f"{'DEMO' if demo else 'REAL'} connected via {self.lib.name}: "
                        f"{'simulated motors' if demo else port} · IDs {ids}")
            self._load_refs()
            self._check()
        elif action == 'disconnect':
            self._stop_all()
            self._close_buses()
            self._set_zero([None] * self.n)
            self._set(connected=False, state='Disconnected', zeroed=False, verified=False, feedback=[None] * self.n,
                      reason='Disconnected.')
            self._event('Disconnected.')
        elif action in ('check', 'ack'):
            self._check(acknowledge=action == 'ack')
        elif action == 'home':
            if self.data['armed'] or self.data['state'] != 'Ready · Stop':
                raise ValueError('Set the reference only while connected, healthy and stopped (Ready · Stop)')
            if v.get('supported') is not True:
                raise ValueError('Confirm the arm is supported at the calibration pose')
            fb = self.data['feedback']
            refs = [dict(method='manual', q_ref_deg=float(a), enc_ref_deg=fb[i]['position_deg'], direction=j['direction'])
                    for i, (a, j) in enumerate(zip(self.cfg['calibration_pose_deg'], self.joints))]
            self._save_refs(refs, 'demo' if self.data['demo'] else 'real')
            self._set(verified=True)  # just set by hand, on this power-up
            self._apply_refs()
            self._event('Manual reference set at the calibration pose ' + str(self.cfg['calibration_pose_deg']) + ' deg.')
            self._check()
        elif action == 'arm':
            self._arm(v)
        elif action == 'calibrate':
            self._cal_start(v, 'calibrate')
        elif action == 'verify':
            self._cal_start(v, 'verify')
        elif action == 'cal_confirm':
            if not (self.cal and self.cal['phase'] == 'confirm'):
                raise ValueError('Nothing to confirm')
            self.cal['confirmed'] = True
        else:
            raise ValueError('Unknown action')

    def _arm(self, v):
        now = time.monotonic()
        if self.data['state'] != 'Ready · Stop' or v.get('supported') is not True:
            raise ValueError('Healthy Stop and supported-arm acknowledgment required')
        if not self.data['zeroed']:
            raise ValueError('Calibrate before arming')
        if self._needs_verify():
            raise ValueError('Verify the saved calibration once after power-up before arming')
        if any(now - t > .25 for t in self.last_reply):
            raise ValueError('Feedback stale; run Check again')
        if now - self.last_ui > self.safety['heartbeat_timeout_s']:
            raise ValueError('Browser heartbeat stale; cannot arm')
        q = self._measured()
        for i, (lo, hi) in enumerate(self.lim):
            if not lo <= q[i] <= hi:
                raise ValueError(f"{self.joints[i]['name']} is outside its limits; move it by hand while stopped")
        for i in range(self.n):
            first = self._foc_q(i, q[i])
            self._gate(i, first, first['mode'])  # first reply reports the previous mode
            self._gate(i, self._foc_q(i, q[i]), 1)
        self._hold_armed(q)

    def _hold_armed(self, q):
        self.q_cmd = list(q)
        self.v_cmd = [0.] * self.n
        self.need_release = True
        self._set(armed=True, state='Armed · holding', q_cmd=[math.degrees(a) for a in q],
                  reason='Holding the measured pose. Release, then hold the deadman (RB / Space) to follow the sim.')
        self._event('ARMED at measured pose ' + ', '.join(f'{math.degrees(a):.1f}' for a in q) + ' deg.')

    def _gravity(self, q):
        """Joint torques (Nm, URDF sign) holding the arm against gravity, from the config masses
        and the manifest's closed-form geometry (same model as web/js/kinematics.js)."""
        a = self.analytic
        m = self.cfg['geometry']['mass_kg']
        l2, l3 = a['L2'] / 1000, a['L3'] / 1000
        qp = [a['sign'][i] * q[i] + a['offset'][i] for i in range(3)]
        s2, s23 = math.sin(qp[1]), math.sin(qp[1] + qp[2])
        t3 = m['forearm'] * l3 / 2 * s23 + m['payload'] * l3 * s23
        t2 = (m['upper_arm'] * l2 / 2 + m['elbow_assembly'] * l2 + (m['forearm'] + m['payload']) * l2) * s2 + t3
        return [0., -9.81 * t2 * a['sign'][1], -9.81 * t3 * a['sign'][2]]

    # ---- calibration / verification state machine ----
    def _verify_plan(self):
        """After a power-up with a saved calibration: touch one J2 contact (prefer a mechanical
        stop over the table) and compare the measured angle with the CAD; then the operator
        confirms the sim's motor-feedback ghost lies on the real arm (J1 and J3)."""
        step = next((s for s in self.plan['steps'] if s['method'] == 'hard_stops'), None)
        if step is None:
            raise ValueError('The calibration plan has no mechanical stop to verify against; recalibrate instead')
        sides = [s for s in ('lo', 'hi') if 'table' not in ' '.join(step['predicted'][s]['contact'])] or ['lo']
        side = sides[0]
        name = self.joints[step['joint']]['name']
        pred = step['predicted'][side]
        return [dict(joint=step['joint'], method='verify_contact', side=side, predicted=step['predicted'],
                     text=f"{name}: touch {pred['deg']:g}° ({pred['contact'][0]}) and compare with the CAD."),
                dict(joint=None, method='confirm_pose',
                     text='Check that the green motor-feedback ghost lies on the real arm (J1 and J3), then confirm.')]

    def _cal_start(self, v, mode):
        if self.data['state'] != 'Ready · Stop' or v.get('supported') is not True:
            raise ValueError('Needs Ready · Stop and the supported-arm acknowledgment')
        if time.monotonic() - self.last_ui > self.safety['heartbeat_timeout_s']:
            raise ValueError('Browser heartbeat stale')
        if mode == 'verify' and not self.data['zeroed']:
            raise ValueError('Nothing to verify: not calibrated')
        c = self.cfg['calibration']
        plan = self.plan['steps'] if mode == 'calibrate' else self._verify_plan()
        steps = [dict(joint=s['joint'], method=s['method'], status='pending', text=s.get('text', s.get('note', '')))
                 for s in plan]
        self.cal = dict(mode=mode, plan=plan, i=0, phase=None, refs=[None] * self.n, targets=None, foc=False,
                        confirmed=False, seek=None, contacts={}, timer=0., settings=SeekSettings(
                            speed_dps=c['seek_speed_dps'], contact_ratio=c['contact_speed_ratio'],
                            contact_lead_deg=c['contact_lead_deg'], contact_time_s=c['contact_time_s'],
                            max_travel_deg=c['max_travel_deg']))
        label = 'Calibrating' if mode == 'calibrate' else 'Verifying'
        self._set(state=label, reason='Follow the calibration panel. Hold the deadman (RB / Space) while it moves.')
        self._set_cal(active=True, mode=mode, steps=steps, message='', awaiting_confirm=False)
        self._event(f'{label} started: ' + ' → '.join(s['method'] if s['joint'] is None else
                                                      f"{self.joints[s['joint']]['name']} ({s['method']})" for s in plan))
        self._cal_next_step()

    def _cal_step(self):
        return self.cal['plan'][self.cal['i']]

    def _cal_status(self, status, **result):
        with self.lock:
            st = self.data['calibration']['steps'][self.cal['i']]
            st['status'] = status
            st.update(result)

    def _cal_next_step(self):
        cal = self.cal
        if cal['i'] >= len(cal['plan']):
            return self._cal_finish()
        step = self._cal_step()
        self._cal_status('running')
        if step['method'] in ('manual', 'confirm_pose'):
            cal['phase'] = 'confirm'
            cal['confirmed'] = False
            msg = (f"{self.joints[step['joint']]['name']}: align it with its reference ({step['reference_deg']:g}° — "
                   'shown in the sim), then confirm.' if step['method'] == 'manual' else step['text'])
            self._set_cal(awaiting_confirm=True, message=msg)
        else:
            cal['phase'] = 'park'
            cal['side'] = step.get('side', 'lo')
            self._set_cal(awaiting_confirm=False,
                          message=f"{self.joints[step['joint']]['name']}: moving to the start of its sweep. Hold the deadman.")

    def _cal_finish(self):
        mode = self.cal['mode']
        if mode == 'calibrate':
            self._save_refs(self.cal['refs'], 'demo' if self.data['demo'] else 'real')
            self._apply_refs()
        q = self._measured()
        self.cal = None
        self._set(verified=True)
        done = 'Calibration complete and saved.' if mode == 'calibrate' else 'Calibration verified for this power-up.'
        self._set_cal(active=False, awaiting_confirm=False, message=done + ' Holding the arm.')
        self._event(done)
        self._hold_armed(q)

    def _cal_fail(self, message):
        mode = self.cal['mode']
        self._event(f'{mode.capitalize()} failed: ' + message)
        self._cal_status('failed', error=message)
        self._stop_all()
        label = 'Calibration failed' if mode == 'calibrate' else 'Verification failed'
        self._set(state=f'{label} — disarmed', reason=message + ' Torque released — support the arm, then Acknowledge.')
        self._set_cal(message=f'{label}: ' + message)

    def _cal_enc(self):
        return [f['position_deg'] for f in self.data['feedback']]

    def _cal_hold(self, rate=None):
        rate = rate or [0.] * self.n
        for i in range(self.n):
            f = self._transact(i, 1, enc=math.radians(self.cal['targets'][i]), rpm=rate[i] / 6, **self._cal_gains(i))
            self._gate(i, f, 1)

    def _cal_tick(self, now, dt):
        cal = self.cal
        step = self._cal_step()
        held = self.deadman and now - self.target_at < self.safety['target_timeout_s']
        c = self.cfg['calibration']

        if cal['phase'] == 'confirm':  # operator step: hold (or stay stopped before FOC)
            if cal['foc']:
                self._cal_hold()
            elif now - self.last_poll >= IDLE_POLL_S:
                self.last_poll = now
                for i in range(self.n):
                    self._gate(i, self._transact(i), 0)
            if cal['confirmed']:
                if step['method'] == 'manual':
                    k, jk = step['joint'], self.joints[step['joint']]
                    enc = self._cal_enc()[k]
                    cal['refs'][k] = dict(method='manual', q_ref_deg=float(step['reference_deg']), enc_ref_deg=enc,
                                          direction=jk['direction'])
                    self._cal_status('done', q_ref_deg=step['reference_deg'], enc_ref_deg=round(enc, 2))
                    self._event(f"{jk['name']}: reference {step['reference_deg']:g}° set at encoder {enc:.2f}°.")
                else:
                    self._cal_status('done')
                    self._event('Operator confirmed the measured pose matches the real arm.')
                self._set_cal(awaiting_confirm=False)
                cal['i'] += 1
                self._cal_next_step()
            return

        k = step['joint']
        jk = self.joints[k]
        if not cal['foc']:  # engage FOC holding every joint where it is
            enc = self._cal_enc()
            cal['targets'] = list(enc)
            for i in range(self.n):
                kw = self._cal_gains(i)
                first = self._transact(i, 1, enc=math.radians(enc[i]), **kw)
                self._gate(i, first, first['mode'])
                self._gate(i, self._transact(i, 1, enc=math.radians(enc[i]), **kw), 1)
            cal['foc'] = True
            self._event('FOC engaged, holding every joint where it is.')

        enc = self._cal_enc()
        speed = [f['rpm'] * 6 for f in self.data['feedback']]  # output deg/s
        step_deg = c['seek_speed_dps'] * dt
        rate = [0.] * self.n

        def ramp(i, goal):
            d = max(-step_deg, min(step_deg, goal - cal['targets'][i]))
            cal['targets'][i] += d
            rate[i] = d / dt if dt else 0.
            return abs(goal - enc[i]) < 0.7 and abs(goal - cal['targets'][i]) < 1e-9

        if cal['phase'] == 'park':  # bring already-calibrated joints to this step's park angles
            if 'park_goal' not in cal:  # in the motor's continuous frame
                cal['park_goal'] = {}
                for key, q_deg in step.get('park_deg', {}).items():
                    i, r = int(key), cal['refs'][int(key)]
                    cal['park_goal'][i] = enc[i] + r['direction'] * (q_deg - joint_angle(r, enc[i], self._centre_deg(i)))
            done = True
            for i, goal in cal['park_goal'].items():
                done = (ramp(i, goal) if held else False) and done
            if done:
                del cal['park_goal']
                cal['phase'] = 'seek'
                cal['seek'] = None
        elif cal['phase'] == 'seek':
            sign = -1 if cal['side'] == 'lo' else 1
            if cal['seek'] is None:
                cal['seek'] = ContactSeek(start_deg=enc[k], motor_sign=jk['direction'] * sign, cfg=cal['settings'])
                self._set_cal(message=f"{jk['name']}: seeking the {'low' if sign < 0 else 'high'}-side contact "
                                      f"(predicted {step['predicted'][cal['side']]['deg']:g}°). Hold the deadman.")
            seek = cal['seek']
            result = seek.step(dt, enc[k], speed[k], active=held)
            if held and not result:
                rate[k] = seek.motor_sign * seek.cfg.speed_dps
            cal['targets'][k] = seek.target_deg
            if result == 'overtravel':
                return self._cal_fail(f"{jk['name']} travelled {seek.cfg.max_travel_deg:g}° without touching anything")
            if result == 'contact':
                cal['contacts'][cal['side']] = enc[k]
                self._event(f"{jk['name']}: {cal['side']} contact at encoder {enc[k]:.2f}°.")
                if step['method'] == 'verify_contact':
                    q_deg = self.data['feedback'][k]['q_deg']
                    pred = step['predicted'][cal['side']]['deg']
                    err = q_deg - pred
                    tol = c.get('verify_tolerance_deg', 3)
                    self._cal_status('running', contact_deg=round(q_deg, 2), predicted_deg=pred, error_deg=round(err, 2))
                    if abs(err) > tol:
                        return self._cal_fail(
                            f"{jk['name']} touched its stop at {q_deg:.1f}° but the CAD puts it at {pred:g}° "
                            f"({err:+.1f}°, tolerance {tol:g}°). The saved calibration does not match this power-up: "
                            'recalibrate (and check that the encoders report an absolute angle at power-up).')
                cal['targets'][k] = enc[k] - seek.motor_sign * c['backoff_deg']
                cal['phase'], cal['timer'] = 'backoff', 0.
        elif cal['phase'] == 'backoff':
            cal['timer'] += dt
            if cal['timer'] > 0.5:
                if step['method'] == 'verify_contact':
                    self._cal_status('done')
                    self._event(f"{jk['name']}: contact within {c.get('verify_tolerance_deg', 3):g}° of the CAD.")
                    cal['i'] += 1
                    self._cal_next_step()
                    if self.cal is None:
                        return
                elif cal['side'] == 'lo':
                    cal['side'], cal['phase'], cal['seek'] = 'hi', 'seek', None
                else:
                    pred = step['predicted']
                    res = solve_two_contacts(jk['direction'], cal['contacts']['lo'], cal['contacts']['hi'],
                                             pred['lo']['deg'], pred['hi']['deg'], c['span_tolerance_deg'])
                    summary = dict(measured_span_deg=round(res['measured_span_deg'], 2),
                                   span_error_deg=round(res['span_error_deg'], 2), q_ref_deg=round(res['q_ref_deg'], 2),
                                   enc_ref_deg=round(res['enc_ref_deg'], 2))
                    if 'error' in res:
                        self._cal_status('failed', **summary)
                        return self._cal_fail(f"{jk['name']}: {res['error']}")
                    ref = dict(method=step['method'], direction=jk['direction'], q_ref_deg=res['q_ref_deg'], enc_ref_deg=res['enc_ref_deg'],
                               stops_enc_deg=res['stops_enc_deg'], predicted_deg=res['predicted_deg'],
                               measured_span_deg=res['measured_span_deg'], span_error_deg=res['span_error_deg'])
                    if step['method'] == 'hard_stops':  # the arm's own body stops it: a real joint limit
                        lo, hi = res['measured_stops_deg']
                        ref['limits_deg'] = [lo + c['limit_margin_deg'], hi - c['limit_margin_deg']]
                        summary['limits_deg'] = [round(x, 2) for x in ref['limits_deg']]
                    cal['refs'][k] = ref
                    self._cal_status('done', **summary)
                    self._event(f"{jk['name']}: span {res['measured_span_deg']:.2f}° "
                                f"({res['span_error_deg']:+.2f}° vs CAD); reference {res['q_ref_deg']:.2f}° at encoder "
                                f"{res['enc_ref_deg']:.2f}°.")
                    cal['phase'] = 'center'
        elif cal['phase'] == 'center':  # park the joint at its reference before the next step
            if held and ramp(k, cal['refs'][k]['enc_ref_deg']):
                cal['i'] += 1
                self._cal_next_step()
                if self.cal is None:
                    return
        if self.cal:
            self._cal_hold(rate)

    def _cal_gains(self, i):
        c = self.cfg['calibration']
        return dict(kp=c['kp'][i], kd=c['kd'][i])

    # ---- armed: trajectory toward the browser's target ----
    def _plan_step(self, goal, dt):
        """Acceleration-limited step of q_cmd toward goal (per joint trapezoid: accelerate, cruise
        at max_speed_dps, brake to arrive). Returns (q, v) without committing them."""
        q_next, v_next = [], []
        for i, (lo, hi) in enumerate(self.lim):
            j = self.joints[i]
            vmax = math.radians(j['max_speed_dps'])
            acc = math.radians(j.get('max_accel_dps2', 180))
            err = min(hi, max(lo, goal[i])) - self.q_cmd[i]
            if dt <= 0:
                q_next.append(self.q_cmd[i])
                v_next.append(self.v_cmd[i])
                continue
            # Fastest speed that can still brake to rest on the goal in steps of acc*dt (the
            # discrete form of sqrt(2*a*d)), and never more than reaches the goal this step.
            brake = math.sqrt((acc * dt / 2) ** 2 + 2 * acc * abs(err)) - acc * dt / 2
            v_want = math.copysign(min(vmax, brake, abs(err) / dt), err)
            v = max(self.v_cmd[i] - acc * dt, min(self.v_cmd[i] + acc * dt, v_want))
            q = self.q_cmd[i] + v * dt
            if not lo <= q <= hi:  # only if the goal jumped and braking could not keep up
                q, v = min(hi, max(lo, q)), 0.
            q_next.append(q)
            v_next.append(v)
        return q_next, v_next

    def _effort(self, q, v, tau):
        """Torque each motor would be asked for (Nm): feed-forward + Kp/Kd terms on the last feedback."""
        out = []
        for i, j in enumerate(self.joints):
            f = self.data['feedback'][i]
            q_meas = math.radians(f['q_deg'])
            dq_meas = j['direction'] * f['rpm'] * math.tau / 60
            out.append(tau[i] + j['kp'] * (q[i] - q_meas) + j['kd'] * (v[i] - dq_meas))
        return out

    def _armed_tick(self, now, dt):
        with self.lock:
            if not self.deadman:
                self.need_release = False
            follow = (self.deadman and not self.need_release and self.target
                      and now - self.target_at < self.safety['target_timeout_s'])
            goal = list(self.target) if follow else list(self.q_cmd)
        q, v = self._plan_step(goal, dt)
        tau = [0.] * self.n
        if self.safety['gravity_ff']:
            cap = self.safety['torque_ff_limit_nm']
            tau = [max(-cap, min(cap, t)) for t in self._gravity(q)]
        # Effort guard: never step further into a predicted effort above the limit (pushing into
        # something, or a pose the motor cannot hold under im6014's 4 Nm trip). Steps that reduce
        # it are always allowed, so the arm can back out.
        limit = self.safety.get('effort_limit_nm', 3.5)
        e_step = self._effort(q, v, tau)
        e_hold = self._effort(self.q_cmd, [0.] * self.n, tau)
        blocked = [i for i in range(self.n) if abs(e_step[i]) > limit and abs(e_step[i]) > abs(e_hold[i]) + 0.05]
        if blocked:
            q, v = list(self.q_cmd), [0.] * self.n
            self._set(effort_limited=f"{self.joints[blocked[0]]['name']} at {abs(e_step[blocked[0]]):.2f} Nm")
        else:
            self._set(effort_limited=None)
        self.q_cmd, self.v_cmd = q, v
        margin = math.radians(self.safety['limit_margin_deg'])
        max_err = math.radians(self.safety['max_tracking_error_deg'])
        for i in range(self.n):
            f = self._foc_q(i, self.q_cmd[i], v[i], tau[i])
            self._gate(i, f, 1)
            qm = self._q(i, f)
            lo, hi = self.lim[i]
            if not lo - margin <= qm <= hi + margin:
                raise RuntimeError(f"{self.joints[i]['name']} measured {math.degrees(qm):.1f}° is past its limit")
            if abs(qm - self.q_cmd[i]) > max_err:
                raise RuntimeError(f"{self.joints[i]['name']} tracking error {math.degrees(qm - self.q_cmd[i]):.1f}° "
                                   f"exceeds {self.safety['max_tracking_error_deg']}°")
        moving = bool(follow)
        self._set(q_cmd=[math.degrees(a) for a in self.q_cmd], moving=moving,
                  state='Armed · moving' if moving else 'Armed · holding',
                  reason='Following the sim target (deadman held).' if moving else
                  'Holding. Hold the deadman (RB / Space) to follow the sim.')

    # ---- control loop ----
    def _tick(self):
        now = time.monotonic()
        dt = min(.05, now - self.last_tick)
        self.last_tick = now
        if self.cal or self.data['armed']:
            if now - self.last_ui > self.safety['heartbeat_timeout_s']:
                raise RuntimeError('Browser heartbeat lost')
            in_foc = self.data['armed'] or (self.cal and self.cal['foc'])
            if in_foc and any(now - t > self.safety['feedback_timeout_s'] for t in self.last_reply):
                raise RuntimeError('Motor feedback stale')
        if self.cal:
            return self._cal_tick(now, dt)
        if self.data['armed']:
            return self._armed_tick(now, dt)
        if self.buses and now - self.last_poll >= IDLE_POLL_S:
            self.last_poll = now
            for i in range(self.n):
                f = self._transact(i)
                if not self.data['state'].startswith(('Fault', 'Timeout', 'Calibration failed', 'Verification failed')):
                    self._gate(i, f, 0, allow_timeout=False)

    def _loop(self):
        hz_t, hz_n = time.monotonic(), 0
        try:
            while not self.closing.is_set():
                start = time.monotonic()
                if self.stop_requested.is_set():
                    was_moving = self.data['armed'] or self.cal is not None
                    confirmed = self._stop_all()
                    self.stop_requested.clear()
                    self._event('Operator Stop; ' + ('confirmed on all joints.' if confirmed else 'NOT confirmed.'))
                    if self.buses:
                        self._set(state='Stopped · re-check required', reason='Check before re-arming.' +
                                  (' Torque released — support the arm.' if was_moving else ''))
                try:
                    generation, action, values = self.jobs.get_nowait()
                except queue.Empty:
                    action = None
                try:
                    if action and generation == self.generation:
                        try:
                            self._handle(action, values)
                        except ValueError as error:  # a refused request; nothing was commanded
                            self._event('Rejected: ' + str(error))
                            self._set(reason=str(error))
                    self._tick()  # any error while commanding is a fault, including ValueError
                except InterruptedError:
                    self._stop_all()
                except Exception as error:
                    if self.buses and not all(getattr(b, 'is_open', True) for b in self.buses):
                        self._close_buses()
                        self._set(connected=False)
                    if not self.data['state'].startswith('Fault'):
                        self._fault(str(error) or type(error).__name__)
                    time.sleep(.1)
                hz_n += 1
                if start - hz_t >= 1:
                    self._set(loop_hz=hz_n / (start - hz_t), cycle_ms=round(1000 * (start - hz_t) / hz_n, 1))
                    hz_t, hz_n = start, 0
                time.sleep(max(0., TICK_S - (time.monotonic() - start)))
        finally:
            self._stop_all()
            self._close_buses()
