"""Serial state machine, independent of ROS; caller holds a lock around every call."""
from dataclasses import dataclass, field
from collections import deque
import math
import threading


FIELDS = ('active requested target c1 c2 pwm elapsed reason seconds encoder_sign '
          'algorithm position_mode goal_turns position_turns p i d ff saturated '
          'kp ki kd antiwindup max_pwm').split()


class EncoderSpeed:
    """Unsigned live RPM, including coast/idle; LAB speeds freeze when a move ends."""
    def __init__(self):
        self.samples = deque()
        self.last = None
        self.totals = (0, 0)
        self.at = -math.inf

    def receive(self, line, now):
        parts = line.split()
        if len(parts) != 5 or parts[0] != b'ENC':
            return None
        try:
            c1, c2, l1, l2 = map(int, parts[1:])
        except ValueError:
            return None
        if not (0 <= c1 <= 0xffffffff and 0 <= c2 <= 0xffffffff and l1 in (0,1) and l2 in (0,1)):
            return None
        counts = (c1, c2)
        deltas = tuple((v-p)&0xffffffff for v,p in zip(counts,self.last or counts))
        if now-self.at > 1 or now <= self.at or any(d > 0x7fffffff for d in deltas):
            self.samples.clear()
            self.totals = (0,0)
        else:
            self.totals = tuple(v+d for v,d in zip(self.totals,deltas))
        self.at, self.last = now, counts
        self.samples.append((now,self.totals))
        while len(self.samples)>2 and self.samples[1][0] <= now-1:
            self.samples.popleft()
        start, baseline = self.samples[0]
        if now == start:
            return None
        return tuple((v-b)*60/(1400*(now-start)) for v,b in zip(self.totals,baseline))


def parse_status(line):
    parts = line.split()
    if len(parts) != 25 or parts[0] != b'LAB1':
        return None
    try:
        row = {key: value.decode('ascii') if key in ('reason', 'algorithm') else float(value)
               for key, value in zip(FIELDS, parts[1:])}
    except (ValueError, UnicodeError):
        return None
    if (any(not math.isfinite(value) for value in row.values() if isinstance(value, float))
            or row['active'] not in (0, 1) or not 0 <= row['pwm'] <= 100
            or row['elapsed'] < 0):
        return None
    return row


@dataclass(frozen=True)
class ControllerConfig:
    sign: int = -1
    tolerance: float = 5.0
    kp: float = .15
    ki: float = .30
    kd: float = .005  # Wire protocol field; PI does not use the derivative term.
    antiwindup: bool = True
    max_pwm: float = 80.0

    def __post_init__(self):
        if type(self.sign) is not int or self.sign not in (-1, 1):
            raise ValueError('encoder_sign must be -1 or 1')
        if type(self.antiwindup) is not bool:
            raise ValueError('antiwindup must be bool')
        for name, low, high in [('kp',0,5),('ki',0,10),('kd',0,1),('max_pwm',20,80),('tolerance',0,180)]:
            value = getattr(self,name)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{name} must be finite and in [{low}, {high}]')
        if self.tolerance == 0:
            raise ValueError('tolerance must be positive')


PARAMETER_DEFAULTS = dict(port='/dev/ttyACM0', publish_signed_encoder=False, encoder_sign=-1, tolerance_deg=5.0,
                          target_rpm=120, run_seconds=5, target_turns=0.0,
                          kp=.15, ki=.30, antiwindup=True, max_pwm=80.0)


def validate_parameters(values):
    """Validate a whole candidate configuration before accepting any parameter update."""
    if not isinstance(values['port'],str) or not values['port'].startswith('/dev/'):
        raise ValueError('port must be a /dev/ serial device path')
    if type(values['publish_signed_encoder']) is not bool:
        raise ValueError('publish_signed_encoder must be bool')
    for name, low, high in [('target_rpm',40,180),('run_seconds',1,60)]:
        if type(values[name]) is not int or not low <= values[name] <= high:
            raise ValueError(f'{name} must be an integer {low}..{high}')
    turns = values['target_turns']
    if (isinstance(turns,bool) or not isinstance(turns,(int,float)) or not math.isfinite(turns)
            or not (turns == 0 or 5/360 <= abs(turns) <= 100)):
        raise ValueError('target_turns: 0 for timed run, otherwise signed 5 degrees..100 turns')
    return ControllerConfig(sign=values['encoder_sign'], tolerance=values['tolerance_deg'],
                            kp=values['kp'], ki=values['ki'], antiwindup=values['antiwindup'],
                            max_pwm=values['max_pwm'])


@dataclass
class Job:
    turns: float
    rpm: int
    timeout: int
    created: float
    config: ControllerConfig = field(default_factory=ControllerConfig)
    done: threading.Event = field(default_factory=threading.Event)
    cancel: bool = False
    start_sequence: int = 0
    stop_sequence: int = 0
    stop_at: float = 0
    stop_reason: str = ''
    row: dict = field(default_factory=dict)
    outcome: str = ''
    disabled: bool = False
    message: str = ''


class ActionEngine:
    def __init__(self, write, sign=-1, tolerance=5.0):
        self.write = write
        self.config = ControllerConfig(sign=sign, tolerance=tolerance)
        self.sent = self.acked = 0
        self.status = None
        self.status_at = -math.inf
        self.job = None
        self.fault = False

    def reserve(self, turns, rpm, timeout, now):
        if (not math.isfinite(turns) or not (turns == 0 or 5/360 <= abs(turns) <= 100)
                or type(rpm) is not int or not 40 <= rpm <= 180
                or type(timeout) is not int or not 1 <= timeout <= 60):
            return None, 'Turns: 0 for timed run, or signed 5 degrees..100 turns; RPM 40..180, time 1..60s.'
        if self.job is not None:
            return None, 'Another goal is still being handled.'
        if self.fault:
            return None, 'Previous stop was not confirmed; reconnect/check board and restart server.'
        if self.status is None or now-self.status_at > 1:
            return None, 'Waiting for fresh LAB1 telemetry from ESP32.'
        if self.status['active'] or self.status['pwm'] != 0:
            return None, 'Board is busy.'
        turns = 0.0 if turns == 0 else turns
        self.job = Job(turns, rpm, timeout, now, config=self.config)
        return self.job, ''

    def send(self, command):
        self.write(command)
        self.sent += 1
        return self.sent

    def stop(self, job, reason, now):
        if job.stop_sequence or job.done.is_set():
            return
        job.stop_reason, job.stop_at = reason, now
        job.stop_sequence = self.send(b'MOTOR STOP\n')
        self.write(b'LAB?\n')

    def finish(self, job, outcome, disabled, message):
        job.outcome, job.disabled, job.message = outcome, disabled, message
        job.done.set()

    def receive(self, line, now):
        if line.strip() == b'ACK MOTOR':
            self.acked = min(self.sent, self.acked+1)
        row = parse_status(line)
        if row is None:
            return None
        self.status, self.status_at = row, now
        job = self.job
        if job is None or job.done.is_set() or not job.start_sequence:
            # A pre-start cancellation can still need a STOP confirmation.
            if job and job.stop_sequence and not job.done.is_set():
                self.confirm_stop(job, row)
            return row
        if job.stop_sequence:
            self.confirm_stop(job, row)
            return row
        if self.acked < job.start_sequence:
            return row  # Reply to a query preceding our start command.
        if job.cancel:
            self.stop(job, 'CANCEL_REQUESTED', now)
            return row
        expected = dict(requested=math.copysign(job.rpm, job.turns), seconds=job.timeout,
                        encoder_sign=job.config.sign, position_mode=int(job.turns != 0), goal_turns=job.turns,
                        kp=job.config.kp, ki=job.config.ki, kd=job.config.kd,
                        antiwindup=int(job.config.antiwindup), max_pwm=job.config.max_pwm)
        if row['algorithm'] != 'PI' or any(abs(row[k]-v) > .00011 for k, v in expected.items()):
            self.stop(job, 'PARAMETER_MISMATCH', now)
            return row
        job.row = row.copy()
        if not row['active']:
            disabled = row['pwm'] == 0
            error = (row['position_turns']-job.turns)*360 if job.turns else 0.0
            success = (row['reason'] == 'DONE' and disabled
                       and (not job.turns or abs(error) <= job.config.tolerance))
            if not disabled:
                self.stop(job, 'DRIVE_NOT_DISABLED', now)
            else:
                self.finish(job, 'SUCCEEDED' if success else 'ABORTED', True,
                            (f"{row['reason']}; encoder error={error:+.2f} deg; "
                             f"tolerance={job.config.tolerance:.1f} deg.") if job.turns else
                            f"{row['reason']}; timed run ended; angular error is not applicable.")
        return row

    def confirm_stop(self, job, row):
        if self.acked >= job.stop_sequence and not row['active'] and row['pwm'] == 0:
            if job.start_sequence:
                job.row = row.copy()
            self.finish(job, 'CANCELED' if job.stop_reason == 'CANCEL_REQUESTED' else 'ABORTED',
                        True, job.stop_reason + '; ESP32 confirmed drive disabled; wheel may coast.')

    def tick(self, now):
        job = self.job
        if job is None or job.done.is_set():
            return
        if job.stop_sequence:
            if now-job.stop_at > 1.5:
                self.fault = True
                self.finish(job, 'ABORTED', False, job.stop_reason + '; STOP NOT CONFIRMED.')
            return
        if job.cancel:
            self.stop(job, 'CANCEL_REQUESTED', now)
        elif now-self.status_at > 1:
            self.stop(job, 'FEEDBACK_TIMEOUT', now)
        elif not job.start_sequence:
            direction = 'FWD' if job.turns >= 0 else 'REV'
            c = job.config
            job.start_sequence = self.send(
                f'MOTOR LAB {job.rpm} {job.timeout} {direction} {c.sign} PI '
                f'{abs(job.turns):.9g} {c.kp:.9g} {c.ki:.9g} {c.kd:.9g} '
                f'{int(c.antiwindup)} {c.max_pwm:.9g} 0 0\n'.encode())
            self.write(b'LAB?\n')
        elif not job.row and now-job.created > 1:
            self.stop(job, 'START_NOT_CONFIRMED', now)
        elif now-job.created > job.timeout+4:
            self.stop(job, 'HOST_TIMEOUT', now)

    def feedback(self, job):
        current = job.row.get('position_turns', 0.0)
        progress = (100*current/job.turns if job.turns else
                    100*job.row.get('elapsed',0.0)/job.timeout)
        return dict(current_turns=current, target_turns=job.turns,
                    remaining_turns=max(0.0, abs(job.turns)-math.copysign(1, job.turns)*current) if job.turns else 0.0,
                    progress_percent=max(0.0, min(100.0, progress)),
                    rpm=(job.row.get('c1', 0.0)+job.row.get('c2', 0.0))/2,
                    pwm_percent=job.row.get('pwm', 0.0),
                    phase='STOPPING' if job.stop_sequence else job.row.get('reason', 'QUEUED'))

    def disconnect(self, message):
        self.fault = True
        if self.job is not None and not self.job.done.is_set():
            self.finish(self.job, 'ABORTED', False, message + '; drive-off not confirmed.')
