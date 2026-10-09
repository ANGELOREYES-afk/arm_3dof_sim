"""Demo actuators: the IM6014 repo's packet-level DemoSerial plus an absolute output encoder and
physical stops, on one shared bus, so calibration and control run end to end without hardware.

Each simulated joint powers up at the calibration start pose, reporting its output angle offset by a
fixed, arbitrary mounting angle (what calibration has to find), and cannot pass the contacts the
CAD predicts for that joint. Contacts that depend on other joints (the forearm on the table) exist
only while those joints are at the calibration plan's park angles. No gravity.
"""
import math
import struct
import time

# Arbitrary but fixed so a saved demo calibration stays valid across bridge restarts.
MOUNTING_DEG = (137.4, -61.9, 98.2)


class SharedDemoBus:
    """One half-duplex RS-485 bus with several simulated motors: a request goes to the motor at
    its ID, and that motor's reply is what the following reads return. No motor, no reply."""

    is_open = True

    def __init__(self, motors):
        self.motors = motors  # {motor_id: DemoSerial}
        self.data = b''
        self.timeout = 0.005
        self.write_timeout = 0.05

    @property
    def in_waiting(self):
        return len(self.data)

    def reset_input_buffer(self):
        self.data = b''

    def write(self, packet):
        motor = self.motors.get(packet[2] & 15)
        if motor is None:
            return len(packet)
        n = motor.write(packet)
        self.data += motor.data
        motor.data = b''
        return n

    def read(self, size):
        data, self.data = self.data[:size], self.data[size:]
        if not data:
            time.sleep(0.001)
        return data

    def close(self):
        self.is_open = False


def demo_bus(sim, codec, joints, plan):
    """All joints on one simulated bus (each motor keeps its own time step)."""
    start = plan['start_pose_deg']
    motors = {}

    def true_deg(i):
        pos = motors[joints[i]['id']].states[joints[i]['id']]['pos']
        return (math.degrees(pos) - MOUNTING_DEG[i]) * joints[i]['direction']

    for i, j in enumerate(joints):
        enc0 = MOUNTING_DEG[i] + j['direction'] * start[i]
        step = next((s for s in plan['steps'] if s['joint'] == i and s['predicted'].get('lo') and s['predicted'].get('hi')), None)
        lim, active = None, None
        if step:
            a, b = (MOUNTING_DEG[i] + j['direction'] * step['predicted'][side]['deg'] for side in ('lo', 'hi'))
            lim = (math.radians(min(a, b)), math.radians(max(a, b)))
            park = {int(k): v for k, v in step.get('park_deg', {}).items()}
            active = lambda park=park: all(abs(true_deg(k) - v) < 3 for k, v in park.items())
        motors[j['id']] = _make(sim, codec, j['id'], math.radians(enc0), lim, active)
    return SharedDemoBus(motors)


def true_angle_deg(bus, joints, i):
    """The simulated joint's true URDF angle (for tests)."""
    pos = bus.motors[joints[i]['id']].states[joints[i]['id']]['pos']
    return (math.degrees(pos) - MOUNTING_DEG[i]) * joints[i]['direction']


def _make(sim, codec, motor_id, pos0, limits, active):
    class StopDemoSerial(sim.DemoSerial):
        def write(self, packet):
            n = super().write(packet)
            st = self.states.get(motor_id)
            if limits and st and not limits[0] <= st['pos'] <= limits[1] and active():
                st['pos'] = min(limits[1], max(limits[0], st['pos']))
                st['rpm'] = 0.
                if len(self.data) == 26:  # re-encode the reply with the clamped state
                    f = list(struct.unpack('<BbBBhhiIHBB', self.data[2:22]))
                    f[5] = 0
                    f[6] = round(st['pos'] / math.tau * codec.RATIO * 32768)
                    body = struct.pack('<BbBBhhiIHBB', *f)
                    self.data = b'\xfc\xee' + body + struct.pack('<I', codec.crc32(body))
            return n

    motor = StopDemoSerial()
    motor.states = {motor_id: dict(mode=0, timeout=1, rpm=0., pos=pos0, torque=0.)}
    return motor
