#!/usr/bin/env python3
"""ROS 2 encoder bridge; read-only unless --enable-control is explicitly selected."""
import argparse
from collections import deque
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import serial
from std_msgs.msg import Bool, Float64, String, UInt32MultiArray
from std_srvs.srv import Trigger


def parse_counts(line):
    fields = line.split()
    if len(fields) != 5 or fields[0] != b'ENC':
        return None
    try:
        c1, c2, level1, level2 = map(int, fields[1:])
    except ValueError:
        return None
    if not (0 <= c1 <= 0xffffffff and 0 <= c2 <= 0xffffffff
            and level1 in (0, 1) and level2 in (0, 1)):
        return None
    return [c1, c2]


class RpmEstimator:
    """Unsigned wheel RPM from per-channel edge counts over about one second."""

    def __init__(self, counts_per_rev=1400.0, window=1.0, max_gap=1.0):
        if any(not math.isfinite(v) or v <= 0 for v in (counts_per_rev, window, max_gap)):
            raise ValueError('RPM estimator settings must be finite and positive')
        self.counts_per_rev = counts_per_rev
        self.window = window
        self.max_gap = max_gap
        self.reset()

    def reset(self):
        self.samples = deque()
        self.last_counts = None
        self.totals = (0, 0)

    def update(self, timestamp, counts):
        counts = tuple(counts)
        if self.samples:
            elapsed = timestamp - self.samples[-1][0]
            deltas = tuple((now - old) & 0xffffffff for now, old in zip(counts, self.last_counts))
            # Small modulo differences handle uint32 wrap; large ones indicate reboot/reset.
            if elapsed <= 0 or elapsed > self.max_gap or any(d > 0x7fffffff for d in deltas):
                self.reset()
            else:
                self.totals = tuple(total + delta for total, delta in zip(self.totals, deltas))
        self.last_counts = counts
        self.samples.append((timestamp, self.totals))
        while len(self.samples) > 2 and self.samples[1][0] <= timestamp - self.window:
            self.samples.popleft()
        start, baseline = self.samples[0]
        elapsed = timestamp - start
        if elapsed <= 0:
            return None
        return tuple((total - initial) * 60 / (self.counts_per_rev * elapsed)
                     for total, initial in zip(self.totals, baseline))


RUN_SECONDS = 5


def parse_motor_status(line):
    fields = line.split()
    if len(fields) != 10 or fields[0] != b'PI3':
        return None
    try:
        active, seconds, sign = int(fields[1]), int(fields[8]), int(fields[9])
        target, c1, c2, pwm, elapsed = map(float, fields[2:7])
        reason = fields[7].decode('ascii')
    except (ValueError, UnicodeError):
        return None
    if (active not in (0, 1) or not 1 <= seconds <= 60 or sign not in (-1, 1)
            or not all(math.isfinite(x) for x in (target, c1, c2, pwm, elapsed))
            or not 0 <= pwm <= 100 or elapsed < 0):
        return None
    return dict(active=bool(active), target=target, seconds=seconds, sign=sign,
                pwm=pwm, elapsed=elapsed, reason=reason)


class MotorCommands:
    """A bounded one-shot command protocol. Zero stops/rearms; positive runs once."""

    def __init__(self, write, logger, publish_goal, publish_state, encoder_sign=-1):
        self.write = write
        self.logger = logger
        self.publish_goal = publish_goal
        self.publish_state = publish_state
        self.encoder_sign = encoder_sign
        self.status = None
        self.status_at = None
        self.encoder_at = None
        self.active = False
        self.confirmed = False
        self.armed = True
        self.started_at = None
        self.expected = None
        self.ready_logged = False

    def stop(self, reason='STOP_REQUESTED', rearm=False):
        self.write(b'MOTOR STOP\n')
        self.active = self.confirmed = False
        self.armed = rearm
        self.publish_goal(0.0)
        self.publish_state(reason)
        self.logger.info(reason)

    def receive(self, rpm):
        if not math.isfinite(rpm):
            self.logger.warning('Rejected: target RPM must be finite.')
            return
        if rpm == 0:
            self.stop(rearm=True)
            return
        if not 40 <= rpm <= 180 or not float(rpm).is_integer():
            self.logger.warning('Rejected: use an integer 40..180 RPM; 0 stops/rearms.')
            return
        if self.active:
            self.logger.warning('Already running; command ignored, 5-second deadline unchanged.')
            return
        if not self.armed:
            self.logger.warning('Send 0 RPM to rearm before another run.')
            return
        now = time.monotonic()
        if (self.status_at is None or self.encoder_at is None
                or now-self.status_at > 1 or now-self.encoder_at > 1):
            self.logger.warning('Not ready: waiting for fresh encoder and PI3 status responses.')
            return
        if self.status['active']:
            self.logger.warning('Board is busy; wait for it to finish or send 0 RPM.')
            return
        self.active, self.confirmed, self.armed = True, False, False
        self.expected = int(rpm)
        self.started_at = now
        self.write(f'MOTOR PI {self.expected} {RUN_SECONDS} FWD {self.encoder_sign}\n'.encode())
        self.publish_goal(float(self.expected))
        self.publish_state('STARTING')
        self.logger.info(f'Requested {self.expected} RPM for {RUN_SECONDS}s; board will confirm.')

    def handle_status(self, status):
        self.status = status
        self.status_at = time.monotonic()
        if self.active:
            if status['active']:
                if (status['target'] != self.expected or status['seconds'] != RUN_SECONDS
                        or status['sign'] != self.encoder_sign):
                    self.stop('PARAMETER_MISMATCH')
                    return
                self.confirmed = True
            elif self.confirmed:
                self.active = False
                self.logger.info(f"Board finished: {status['reason']}. Send 0 RPM before another run.")
            else:
                # A status reply requested before the command can still be in flight.
                self.publish_state('STARTING')
                return
        self.publish_goal(abs(status['target']) if status['active'] else 0.0)
        self.publish_state(status['reason'])
        if (not status['active'] and not self.ready_logged and self.encoder_at is not None
                and self.status_at-self.encoder_at <= 1):
            self.ready_logged = True
            self.logger.info('READY: publish once to /motor/target_rpm; 0 stops/rearms.')

    def tick(self):
        if not self.active:
            return
        now = time.monotonic()
        if (now-self.encoder_at > 1 or now-self.status_at > 1):
            self.stop('FEEDBACK_TIMEOUT')
        elif not self.confirmed and now-self.started_at > 1:
            self.stop('START_NOT_CONFIRMED')
        elif now-self.started_at > RUN_SECONDS+4:
            self.stop('HOST_TIMEOUT')


class EncoderPublisher(Node):
    def __init__(self, port, enable_control=False, encoder_sign=-1):
        super().__init__('esp32_encoder')
        self.port = port
        self.publisher = self.create_publisher(UInt32MultiArray, '/motor/encoder_counts', 10)
        self.rpm = self.create_publisher(Float64, '/motor/rpm', 10)
        self.c1_rpm = self.create_publisher(Float64, '/motor/c1_rpm', 10)
        self.c2_rpm = self.create_publisher(Float64, '/motor/c2_rpm', 10)
        self.rpm_valid = self.create_publisher(Bool, '/motor/rpm_valid', 10)
        self.commands = None
        self.motor_commands_sent = 0
        self.motor_commands_acked = 0
        self.latest_motor_status = None
        self.status_ack_sequence = 0
        self.stop_in_progress = False
        self.status_sent_at = 0.0
        if enable_control:
            self.setpoint = self.create_publisher(Float64, '/motor/setpoint', 10)
            self.pwm = self.create_publisher(Float64, '/motor/pwm', 10)
            self.state = self.create_publisher(String, '/motor/state', 10)
            self.commands = MotorCommands(
                self.write_line, self.get_logger(),
                lambda value: self.setpoint.publish(Float64(data=value)),
                lambda value: self.state.publish(String(data=value)), encoder_sign)
            self.target_subscription = self.create_subscription(
                Float64, '/motor/target_rpm', self.on_target, 1)
            self.stop_service = self.create_service(Trigger, '/motor/stop', self.on_stop)
            self.get_logger().info('Control enabled: positive RPM runs once for 5s; 0 stops/rearms.')
            self.get_logger().info('Stop service: /motor/stop confirms drive-off from the board.')
        self.speed = RpmEstimator()
        self.buffer = b''
        self.waiting = False
        self.sent_at = 0.0
        self.timer = self.create_timer(0.02, self.poll)
        self.get_logger().info(f'Reading {self.port.port} -> /motor/encoder_counts and /motor/rpm')
        self.get_logger().info('Wheel RPM magnitude; 1400 edges/rev/channel, about 1 second averaging.')

    def write_line(self, command):
        if self.port.write(command) != len(command):
            raise serial.SerialTimeoutException('Incomplete serial write')
        if command.startswith(b'MOTOR '):
            self.motor_commands_sent += 1

    def on_target(self, message):
        if self.stop_in_progress:
            self.get_logger().warning('Stop service is in progress; target command ignored.')
            return
        self.commands.receive(message.data)

    def on_stop(self, request, response):
        # A bounded serial transaction on the same executor thread. Calling poll()
        # keeps telemetry flowing without waiting on another ROS callback/future.
        # No motor-start callbacks execute until this callback returns.
        self.stop_in_progress = True
        response.success = False
        try:
            self.commands.stop('SERVICE_STOP_REQUESTED', rearm=False)
            stop_sequence = self.motor_commands_sent
            self.write_line(b'PI?\n')
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                self.poll()
                status = self.latest_motor_status
                if (self.status_ack_sequence >= stop_sequence and status is not None
                        and not status['active'] and status['pwm'] == 0):
                    response.success = True
                    response.message = 'ESP32 confirmed drive disabled (PWM=0%). Wheel may coast.'
                    self.get_logger().info('Stop service confirmed drive-off; send 0 before the next run.')
                    return response
                time.sleep(0.01)
            response.message = 'Stop sent, but no matching ACK and fresh drive-off status within 1.5s.'
        except serial.SerialException as exc:
            response.message = f'Stop could not be confirmed: {exc}'
        finally:
            self.stop_in_progress = False
            self.commands.armed = False
        return response

    def poll(self):
        self.buffer += self.port.read(min(self.port.in_waiting, 4096))
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            counts = parse_counts(line)
            if line.strip() == b'ACK MOTOR' and self.motor_commands_acked < self.motor_commands_sent:
                self.motor_commands_acked += 1
            if self.commands is not None:
                status = parse_motor_status(line)
                if status is not None:
                    self.latest_motor_status = status
                    # Remember how many command ACKs preceded this status in the stream.
                    # A cached pre-stop reply can never confirm a new stop request.
                    self.status_ack_sequence = self.motor_commands_acked
                    self.commands.handle_status(status)
                    self.pwm.publish(Float64(data=status['pwm']))
            if counts is not None:
                if self.commands is not None:
                    self.commands.encoder_at = time.monotonic()
                self.publisher.publish(UInt32MultiArray(data=counts))
                rpm = self.speed.update(time.monotonic(), counts)
                self.rpm_valid.publish(Bool(data=rpm is not None))
                if rpm is not None:
                    self.c1_rpm.publish(Float64(data=rpm[0]))
                    self.c2_rpm.publish(Float64(data=rpm[1]))
                    self.rpm.publish(Float64(data=(rpm[0] + rpm[1]) / 2))
                self.waiting = False
        if len(self.buffer) > 8192:
            self.buffer = b''
        now = time.monotonic()
        if self.waiting and now - self.sent_at > 1.0:
            self.get_logger().warning('No encoder response; RPM unavailable, not zero.')
            self.speed.reset()
            self.rpm_valid.publish(Bool(data=False))
            self.buffer = b''
            self.waiting = False
        if not self.waiting and now - self.sent_at >= 0.2:
            self.write_line(b'ENC?\n')
            self.sent_at = now
            self.waiting = True
        if self.commands is not None:
            self.commands.tick()
            if now - self.status_sent_at >= 0.2:
                self.write_line(b'PI?\n')
                self.status_sent_at = now


def main():
    parser = argparse.ArgumentParser(description='Encoder/RPM bridge; optionally enable bounded motor commands.')
    parser.add_argument('--enable-control', action='store_true', help='Accept /motor/target_rpm commands, 5 seconds each')
    parser.add_argument('--encoder-sign', type=int, choices=(-1, 1), default=-1)
    parser.add_argument('--port', default='/dev/ttyACM0')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    port = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=1, exclusive=True)
    port.dtr = port.rts = False
    port.port = args.port
    try:
        port.open()
        port.reset_input_buffer()
        node = EncoderPublisher(port, args.enable_control, args.encoder_sign)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except serial.SerialException as exc:
        raise SystemExit(f'Serial error: {exc}') from exc
    finally:
        if args.enable_control and port.is_open:
            try:
                port.write(b'MOTOR STOP\n')
            except serial.SerialException:
                pass  # Board-side time limit still applies if USB is disconnected.
        port.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
