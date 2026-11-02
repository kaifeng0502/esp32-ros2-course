"""Pure signed encoder to output-shaft joint conversion, independent of ROS."""
import math


def parse_encoder_state(line):
    parts = line.split()
    if len(parts) != 5 or parts[0] != b'ENC2':
        return None
    try:
        boot, uptime, c1, c2 = map(int,parts[1:])
    except ValueError:
        return None
    if (not 0 <= boot <= 0xffffffff or not 0 <= uptime < 2**63
            or not all(-2**63 <= c < 2**63 for c in (c1,c2))):
        return None
    return boot, uptime, c1, c2


class WheelEstimator:
    def __init__(self, counts_per_rev=1400.0, encoder_sign=-1):
        if (not math.isfinite(counts_per_rev) or counts_per_rev <= 0
                or type(encoder_sign) is not int or encoder_sign not in (-1,1)):
            raise ValueError('counts_per_rev must be positive; encoder_sign must be -1 or 1')
        self.counts_per_rev = counts_per_rev
        self.sign = encoder_sign
        self.boot = None
        self.origin = None
        self.previous = None
        self.received_at = None

    def update(self, state, now):
        boot, uptime, c1, c2 = state
        first = self.origin is None or boot != self.boot
        restarted = self.origin is not None and boot != self.boot
        if not first and uptime <= self.previous[0]:
            return None  # Duplicate/out-of-order reply must not create a false velocity.
        if first:
            self.boot, self.origin = boot, c1+c2
        scale = self.sign * 2*math.pi/(2*self.counts_per_rev)
        # Subtract integer counts before floating conversion, preserving precision.
        position = ((c1+c2)-self.origin)*scale
        velocity = None
        if not first and now-self.received_at <= 1:
            dt = (uptime-self.previous[0])/1e6
            velocity = ((c1+c2)-self.previous[1])*scale/dt
        self.previous = uptime,c1+c2
        self.received_at = now
        return dict(position=position,velocity=velocity,restarted=restarted)

    def fresh(self,now):
        return self.received_at is not None and now-self.received_at <= 1
