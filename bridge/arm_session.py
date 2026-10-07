"""Three-joint IM6014 session: one serial-owner thread streaming joint-space position targets.

The packet codec, serial transaction, health gate and packet-level simulator are imported
unchanged from im6014Development (spin_5rpm.command/exchange/healthy, motor_session.DemoSerial).
This module adds what that single-motor bench app leaves to separate integration: several
motor IDs, a per-session joint zero, joint limits, rate-limited continuous position streaming,
and an operator deadman. It keeps the bench app's conventions: starts disconnected and
disarmed, Stop/status only until armed, every FOC packet sets the firmware watchdog, any fault
or stale heartbeat/feedback latches Stop and disarm, and nothing resumes on its own.

Stop releases torque. A gravity-loaded arm will fall; support it.
"""
from collections import deque
import copy
import math
import queue
import threading
import time

TICK_S = 0.01
IDLE_POLL_S = 0.05


class ArmSession:
    def __init__(self, cfg, codec, sim):
        self.cfg = cfg
        self.codec, self.sim = codec, sim  # spin_5rpm, motor_session modules
        self.joints = cfg['joints']
        self.safety = cfg['safety']
        self.n = len(self.joints)
        self.lim = [(math.radians(j['min_deg']), math.radians(j['max_deg'])) for j in self.joints]
        self.lock = threading.RLock()
        self.jobs = queue.Queue()
        self.stop_requested = threading.Event()
        self.closing = threading.Event()
        self.generation = 0
        self.buses = []        # per joint; joints on the same port share one serial object
        self.zero = None       # motor output position (rad) at joint zero, per joint
        self.q_cmd = None
        self.target = None     # latest browser target (rad), its time and deadman state
        self.target_at = 0.
        self.deadman = False
        self.last_ui = 0.
        self.last_reply = [0.] * self.n
        self.last_poll = 0.
        self.last_tick = time.monotonic()
        self.started = time.monotonic()
        self.events = deque(maxlen=300)
        self.event_seq = 0
        self.data = dict(state='Disconnected', reason='Connect to the motors (demo or real).', connected=False,
                         demo=False, armed=False, moving=False, zeroed=False, port=cfg['bus']['port'],
                         baud=cfg['bus']['baud'], tx=0, rx=0, crc_errors=0, reply_timeouts=0, loop_hz=0.,
                         feedback=[None] * self.n, q_cmd=None)
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

    def _q(self, i, f):
        if self.zero is None:
            return None
        return self.joints[i]['direction'] * (f['position_rad'] - self.zero[i])

    def _transact(self, i, mode=0, watchdog=True, q=0., vel=0., tau=0.):
        if mode and (self.stop_requested.is_set() or self.closing.is_set()):
            raise InterruptedError('Stop requested')
        j = self.joints[i]
        if mode:
            s = j['direction']
            packet = self.codec.command(j['id'], 1, rpm=s * vel * 60 / math.tau, kd=j['kd'], torque=s * tau,
                                        position=self.zero[i] + s * q, kp=j['kp'])
        else:
            packet = self.codec.command(j['id'], 0, watchdog=watchdog)
        self._set(tx=self.data['tx'] + 1)
        try:
            f = self.codec.exchange(self.buses[i], packet, j['id'])
        except Exception as error:
            if 'CRC' in str(error):
                self._set(crc_errors=self.data['crc_errors'] + 1)
            if 'deadline' in str(error):
                self._set(reply_timeouts=self.data['reply_timeouts'] + 1)
            raise RuntimeError(f"{j['name']} (ID {j['id']}): {error}") from None
        now = time.monotonic()
        self.last_reply[i] = now
        q_meas = self._q(i, f)
        fb = dict(mode=f['mode'], timeout=f['timeout'], position_deg=math.degrees(f['position_rad']),
                  q_deg=None if q_meas is None else math.degrees(q_meas), rpm=f['rpm'], torque_nm=f['torque_nm'],
                  voltage=f['voltage'], driver_c=f['driver_c'], winding_c=f['winding_c'],
                  faults=self.sim.fault_text(f['errors'], f['warnings']), at=now)
        with self.lock:
            self.data['rx'] += 1
            self.data['feedback'][i] = fb
        return f

    def _gate(self, i, f, mode, allow_timeout=False):
        try:
            self.codec.healthy(f, mode, allow_timeout=allow_timeout)
        except RuntimeError as error:
            raise RuntimeError(f"{self.joints[i]['name']}: {error}") from None

    def _stop_all(self):
        """Stop every joint; True only if each confirmed mode 0."""
        self.q_cmd = None
        self._set(armed=False, moving=False, q_cmd=None)
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
        self._stop_all()
        self._set(state='Fault — disarmed', reason=message)
        self._event('DISARMED: ' + message)

    def _open(self, demo, port, baud):
        buses, shared = [], {}
        try:
            for j in self.joints:
                if demo:
                    # One simulator per joint so each gets its own time step.
                    bus = self.sim.DemoSerial()
                    bus.states = {j['id']: dict(mode=0, timeout=1, rpm=0., pos=0., torque=0.)}
                else:
                    p = j.get('port', port)
                    if p not in shared:
                        import serial  # same open sequence as motor_session.MotorSession
                        s = serial.Serial(port=None, baudrate=baud, timeout=0.005, write_timeout=0.05, exclusive=True)
                        s.port = p
                        s.dtr = s.rts = False
                        shared[p] = s
                        s.open()
                    bus = shared[p]
                buses.append(bus)
        except Exception:
            for s in shared.values():
                s.close()
            raise
        return buses

    def _close_buses(self):
        for bus in {id(b): b for b in self.buses}.values():
            try:
                bus.close()
            except Exception:
                pass
        self.buses = []

    def _check(self, acknowledge=False):
        if not self.buses:
            raise ValueError('Connect first')
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
                      '. Support the arm, then Acknowledge timeout.')
            return
        self._set(state='Ready · Stop', reason='Healthy feedback, all joints stopped.' +
                  ('' if self.zero else ' Pose the arm at the calibration pose, then Set home.'))

    def _measured(self):
        fb = self.data['feedback']
        return [math.radians(f['q_deg']) for f in fb]

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
            self.zero = None
            self._set(state='Connecting', reason='Opening adapter, sending Stop only.', demo=demo, port=port,
                      baud=baud, zeroed=False, feedback=[None] * self.n, tx=0, rx=0, crc_errors=0, reply_timeouts=0)
            self.buses = self._open(demo, port, baud)
            self._set(connected=True)
            self._event(f"{'DEMO' if demo else 'REAL'} connected: {'simulated motors' if demo else port} · IDs {ids}")
            self._check()
            if demo:
                self._set_home()  # simulated motors power up at 0 = the calibration pose
        elif action == 'disconnect':
            self._stop_all()
            self._close_buses()
            self.zero = None
            self._set(connected=False, state='Disconnected', zeroed=False, feedback=[None] * self.n,
                      reason='Disconnected. Reconnect needs a new check, home and arm.')
            self._event('Disconnected.')
        elif action in ('check', 'ack'):
            self._check(acknowledge=action == 'ack')
        elif action == 'home':
            if self.data['armed'] or not self.data['state'].startswith('Ready'):
                raise ValueError('Set home only while connected, healthy and stopped (Ready · Stop)')
            if v.get('supported') is not True:
                raise ValueError('Confirm the arm is supported at the calibration pose')
            self._set_home()
        elif action == 'arm':
            self._arm(v)
        else:
            raise ValueError('Unknown action')

    def _set_home(self):
        fb = self.data['feedback']
        if any(f is None for f in fb):
            raise ValueError('No feedback yet')
        pose = [math.radians(a) for a in self.cfg['calibration_pose_deg']]
        self.zero = [math.radians(fb[i]['position_deg']) - self.joints[i]['direction'] * pose[i] for i in range(self.n)]
        with self.lock:
            for i, f in enumerate(fb):
                f['q_deg'] = self.cfg['calibration_pose_deg'][i]
            self.data['zeroed'] = True
            if self.data['state'] == 'Ready · Stop':
                self.data['reason'] = 'Home set. Arm to hold this pose; hold the deadman to move.'
        self._event('Home set at calibration pose ' + str(self.cfg['calibration_pose_deg']) + ' deg.')

    def _arm(self, v):
        now = time.monotonic()
        if self.data['state'] != 'Ready · Stop' or v.get('supported') is not True:
            raise ValueError('Healthy Stop and supported-arm acknowledgment required')
        if not self.zero:
            raise ValueError('Set home before arming')
        if any(now - t > .25 for t in self.last_reply):
            raise ValueError('Feedback stale; run Check again')
        if now - self.last_ui > self.safety['heartbeat_timeout_s']:
            raise ValueError('Browser heartbeat stale; cannot arm')
        q = self._measured()
        for i, (lo, hi) in enumerate(self.lim):
            if not lo <= q[i] <= hi:
                raise ValueError(f"{self.joints[i]['name']} is outside its limits; move it by hand while stopped")
        self.q_cmd = list(q)
        for i in range(self.n):
            first = self._transact(i, 1, q=q[i])
            self._gate(i, first, first['mode'])  # first reply reports the previous mode
            self._gate(i, self._transact(i, 1, q=q[i]), 1)
        self._set(armed=True, state='Armed · holding', q_cmd=[math.degrees(a) for a in q],
                  reason='Holding the measured pose. Hold the deadman (RB / Space) to follow the sim.')
        self._event('ARMED at measured pose ' + ', '.join(f'{math.degrees(a):.1f}' for a in q) + ' deg.')

    def _gravity(self, q):
        """Joint torques (Nm) that hold the arm against gravity, from the config masses."""
        g = self.cfg['geometry']
        m = g['mass_kg']
        l2, l3 = g['upper_arm_mm'] / 1000, g['forearm_mm'] / 1000
        s2, s23 = math.sin(q[1]), math.sin(q[1] + q[2])
        t3 = m['forearm'] * l3 / 2 * s23 + m['payload'] * l3 * s23
        t2 = (m['upper_arm'] * l2 / 2 + m['elbow_motor'] * l2 + (m['forearm'] + m['payload']) * l2) * s2 + t3
        return [0., -9.81 * t2, -9.81 * t3]

    def _tick(self):
        now = time.monotonic()
        dt = min(.05, now - self.last_tick)
        self.last_tick = now
        if self.data['armed']:
            if now - self.last_ui > self.safety['heartbeat_timeout_s']:
                raise RuntimeError('Browser heartbeat lost')
            if any(now - t > self.safety['feedback_timeout_s'] for t in self.last_reply):
                raise RuntimeError('Motor feedback stale')
            with self.lock:
                follow = self.deadman and self.target and now - self.target_at < self.safety['target_timeout_s']
                goal = list(self.target) if follow else list(self.q_cmd)
            vel = [0.] * self.n
            for i, (lo, hi) in enumerate(self.lim):
                step = math.radians(self.joints[i]['max_speed_dps']) * dt
                new = self.q_cmd[i] + max(-step, min(step, min(hi, max(lo, goal[i])) - self.q_cmd[i]))
                vel[i] = (new - self.q_cmd[i]) / dt if dt else 0.
                self.q_cmd[i] = new
            tau = [0.] * self.n
            if self.safety['gravity_ff']:
                cap = self.safety['torque_ff_limit_nm']
                tau = [max(-cap, min(cap, t)) for t in self._gravity(self.q_cmd)]
            margin = math.radians(self.safety['limit_margin_deg'])
            max_err = math.radians(self.safety['max_tracking_error_deg'])
            for i in range(self.n):
                f = self._transact(i, 1, q=self.q_cmd[i], vel=vel[i], tau=tau[i])
                self._gate(i, f, 1)
                q = self._q(i, f)
                lo, hi = self.lim[i]
                if not lo - margin <= q <= hi + margin:
                    raise RuntimeError(f"{self.joints[i]['name']} measured {math.degrees(q):.1f}° is past its limit")
                if abs(q - self.q_cmd[i]) > max_err:
                    raise RuntimeError(f"{self.joints[i]['name']} tracking error {math.degrees(q - self.q_cmd[i]):.1f}° "
                                       f"exceeds {self.safety['max_tracking_error_deg']}°")
            moving = bool(follow)
            self._set(q_cmd=[math.degrees(a) for a in self.q_cmd], moving=moving,
                      state='Armed · moving' if moving else 'Armed · holding',
                      reason='Following the sim target at the configured joint speeds (deadman held).' if moving else
                      'Holding. Hold the deadman (RB / Space) to follow the sim.')
        elif self.buses and now - self.last_poll >= IDLE_POLL_S:
            self.last_poll = now
            for i in range(self.n):
                f = self._transact(i)
                if not self.data['state'].startswith(('Fault', 'Timeout')):
                    self._gate(i, f, 0, allow_timeout=False)

    def _loop(self):
        hz_t, hz_n = time.monotonic(), 0
        try:
            while not self.closing.is_set():
                start = time.monotonic()
                if self.stop_requested.is_set():
                    was_armed = self.data['armed']
                    confirmed = self._stop_all()
                    self.stop_requested.clear()
                    self._event('Operator Stop; ' + ('confirmed on all joints.' if confirmed else 'NOT confirmed.'))
                    if self.buses:
                        self._set(state='Stopped · re-check required', reason='Check before re-arming.' +
                                  (' Torque released — support the arm.' if was_armed else ''))
                try:
                    generation, action, values = self.jobs.get_nowait()
                except queue.Empty:
                    action = None
                try:
                    if action and generation == self.generation:
                        self._handle(action, values)
                    self._tick()
                except InterruptedError:
                    self._stop_all()
                except ValueError as error:
                    self._event('Rejected: ' + str(error))
                    self._set(reason=str(error))
                except Exception as error:
                    if self.buses and not all(getattr(b, 'is_open', True) for b in self.buses):
                        self._close_buses()
                        self._set(connected=False)
                    if not self.data['state'].startswith('Fault'):
                        self._fault(str(error))
                    time.sleep(.1)
                hz_n += 1
                if start - hz_t >= 1:
                    self._set(loop_hz=hz_n / (start - hz_t))
                    hz_t, hz_n = start, 0
                time.sleep(max(0., TICK_S - (time.monotonic() - start)))
        finally:
            self._stop_all()
            self._close_buses()
