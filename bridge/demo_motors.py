"""Demo actuators: im6014Development's packet-level DemoSerial plus an absolute output encoder
and physical stops, so the calibration can run end to end without hardware.

Each simulated joint powers up at the calibration start pose, reporting its output angle offset by a
fixed, arbitrary mounting angle (what calibration has to find), and cannot pass the contacts the
CAD predicts for that joint. Contacts that depend on other joints (the forearm on the table) exist
only while those joints are at the calibration plan's park angles. No gravity.
"""
import math
import struct

# Arbitrary but fixed so a saved demo calibration stays valid across bridge restarts.
MOUNTING_DEG = (137.4, -61.9, 98.2)


def demo_buses(sim, codec, joints, plan):
    """One simulated bus per joint (each with its own time step), with stops from the plan.
    A step's contacts only exist while the joints it parks are at their park angles."""
    start = plan['start_pose_deg']
    buses = []

    def true_deg(i):
        pos = buses[i].states[joints[i]['id']]['pos']
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
        buses.append(_make(sim, codec, j['id'], math.radians(enc0), lim, active))
    return buses


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

    bus = StopDemoSerial()
    bus.states = {motor_id: dict(mode=0, timeout=1, rpm=0., pos=pos0, torque=0.)}
    return bus
