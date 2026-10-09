"""Joint calibration for absolute-output-encoder IM6014 joints. Pure logic: no I/O, no serial.

Each joint maps its output-encoder angle to the URDF joint angle with

    q = centre + wrap(q_ref + dir * (enc - enc_ref) - centre)       wrap to (-180, 180] degrees

where centre is the middle of the joint's limits. That is unambiguous for any joint whose travel
is under 360 deg, whether the actuator reports a single-turn or multi-turn absolute angle at
power-up; the one excluded 360 - travel slice sits opposite the middle of the limits, so pick
limits that put it where the joint never goes. Calibration finds enc_ref for a known q_ref:

  hard_stops / contacts   seek a contact in each direction; the CAD predicts the joint angle at
                          each contact, so enc_ref = midpoint of the two measured encoder angles
                          and q_ref = midpoint of the predicted angles. The measured span must
                          match the predicted span, which catches a wrong ID, a missing stop, a
                          false stall, or CAD that differs from the build.
  manual                  the joint is aligned by eye to its reference angle and confirmed.
"""
from dataclasses import dataclass, field
import math


def wrap(a):
    """Angle in degrees wrapped to (-180, 180]."""
    return a - 360. * math.ceil((a - 180.) / 360.)


def joint_angle(ref, enc_deg, centre_deg=None):
    """URDF joint angle (deg) from an absolute output-encoder angle (deg), within +-180 deg of
    centre_deg (default: the reference angle)."""
    q = ref['q_ref_deg'] + ref['direction'] * wrap(enc_deg - ref['enc_ref_deg'])
    if centre_deg is None:
        return q
    return centre_deg + wrap(q - centre_deg)


@dataclass
class SeekSettings:
    speed_dps: float = 10.         # commanded joint speed while seeking
    contact_ratio: float = 0.3     # measured speed below this share of the commanded speed ...
    contact_lead_deg: float = 2.   # ... while the target leads by at least this much ...
    contact_time_s: float = 0.2    # ... for this long = contact
    settle_s: float = 0.3          # ignore the start of each move
    max_travel_deg: float = 400.   # give up if nothing is touched within this travel


@dataclass
class ContactSeek:
    """Ramp a position target in one direction until the joint stops moving against something.

    Works in the motor's own output frame (degrees), so it runs before the joint is calibrated.
    motor_sign is +1/-1: the motor direction that moves the joint the way we want.
    """
    start_deg: float
    motor_sign: int
    cfg: SeekSettings = field(default_factory=SeekSettings)
    target_deg: float = None
    elapsed: float = 0.
    slow_for: float = 0.
    result: str = None

    def __post_init__(self):
        self.target_deg = self.start_deg

    def step(self, dt, enc_deg, speed_dps, active=True):
        """Advance by dt. Returns 'contact', 'overtravel' or None; target_deg is the next command."""
        if self.result or not active:  # deadman released: hold the target, restart the stall timer
            self.slow_for = 0.
            return self.result
        self.elapsed += dt
        lead = (self.target_deg - enc_deg) * self.motor_sign
        slow = abs(speed_dps) < self.cfg.contact_ratio * self.cfg.speed_dps
        if self.elapsed > self.cfg.settle_s and slow and lead >= self.cfg.contact_lead_deg:
            self.slow_for += dt
        else:
            self.slow_for = 0.
        if self.slow_for >= self.cfg.contact_time_s:
            self.result = 'contact'
            self.target_deg = enc_deg  # stop pushing
        elif abs(enc_deg - self.start_deg) > self.cfg.max_travel_deg:
            self.result = 'overtravel'
            self.target_deg = enc_deg
        else:
            self.target_deg += self.motor_sign * self.cfg.speed_dps * dt
        return self.result


def solve_two_contacts(direction, enc_lo_deg, enc_hi_deg, predicted_lo_deg, predicted_hi_deg, tolerance_deg):
    """Reference from the encoder angles measured at the low-side and high-side contacts.

    enc_lo is where the joint stopped while moving toward negative q, enc_hi toward positive q,
    both in the motor's continuous frame from the same session.
    """
    measured_span = direction * (enc_hi_deg - enc_lo_deg)
    predicted_span = predicted_hi_deg - predicted_lo_deg
    q_ref = (predicted_lo_deg + predicted_hi_deg) / 2
    out = dict(direction=direction, q_ref_deg=q_ref, enc_ref_deg=(enc_lo_deg + enc_hi_deg) / 2,
               stops_enc_deg=[enc_lo_deg, enc_hi_deg], predicted_deg=[predicted_lo_deg, predicted_hi_deg],
               measured_span_deg=measured_span, span_error_deg=measured_span - predicted_span,
               measured_stops_deg=[q_ref - measured_span / 2, q_ref + measured_span / 2])
    if measured_span <= 0:
        out['error'] = 'contacts came in the wrong order; check the joint direction sign'
    elif abs(measured_span - predicted_span) > tolerance_deg:
        out['error'] = (f'measured span {measured_span:.1f}° vs {predicted_span:.1f}° predicted by the CAD. '
                        'Check the motor ID and direction, that nothing else touched, and that the build '
                        'matches the CAD at these contacts.')
    return out
