#!/usr/bin/env python3
"""ROS 2 Action adapter. The timer alone owns normal serial I/O."""
import argparse
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import serial
from std_msgs.msg import Bool, Float64, String
from motor_action_interfaces.action import MoveWheel
from motor_action_interfaces.msg import EncoderState
from motor_action_core import ActionEngine, EncoderSpeed, PARAMETER_DEFAULTS, validate_parameters
from wheel_joint_core import parse_encoder_state


class MotorActionServer(Node):
    def __init__(self, port, sign=None, tolerance=None, device=None):
        super().__init__('motor_action_server')
        self.lock = threading.RLock()
        defaults = dict(PARAMETER_DEFAULTS)
        for name, value in [('port',device),('encoder_sign',sign),('tolerance_deg',tolerance)]:
            if value is not None:
                defaults[name] = value
        for name, value in defaults.items():
            self.declare_parameter(name, value, descriptor=ParameterDescriptor(
                read_only=name in ('port','publish_signed_encoder')))
        settings = {name: self.get_parameter(name).value for name in defaults}
        config = validate_parameters(settings)
        self.port = port
        self.port.port = settings['port']
        self.port.open()
        self.port.reset_input_buffer()
        self.engine = ActionEngine(self.write_line)
        self.engine.config = config
        self.signed_enabled = settings['publish_signed_encoder']
        self.signed_publisher = self.create_publisher(EncoderState,'/motor/encoder_signed',10)
        self.signed_started_at = time.monotonic()
        self.signed_received = False
        self.signed_warned = False
        self.add_on_set_parameters_callback(self.check_parameters)
        self.add_post_set_parameters_callback(self.apply_parameters)
        self.speed = EncoderSpeed()
        self.buffer = b''
        self.query_at = -math.inf
        self.ready = False
        self.telemetry = {name: self.create_publisher(Float64, '/motor/'+name, 10)
                          for name in ('rpm', 'c1_rpm', 'c2_rpm', 'pwm', 'setpoint', 'requested_rpm', 'position_turns')}
        self.valid = self.create_publisher(Bool, '/motor/rpm_valid', 10)
        self.state = self.create_publisher(String, '/motor/state', 10)
        self.server = ActionServer(
            self, MoveWheel, '/motor/move_wheel', execute_callback=self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(.02, self.poll)
        self.get_logger().info('Waiting for ESP32 LAB1 status. Opening this server does not start the motor.')

    def write_line(self, line):
        if self.port.write(line) != len(line):
            raise serial.SerialTimeoutException('Incomplete serial write')

    def poll(self):
        with self.lock:
            try:
                now = time.monotonic()
                self.engine.tick(now)
                self.buffer += self.port.read(min(self.port.in_waiting, 4096))
                while b'\n' in self.buffer:
                    line, self.buffer = self.buffer.split(b'\n', 1)
                    signed = parse_encoder_state(line)
                    if self.signed_enabled and signed is not None:
                        message = EncoderState()
                        message.header.stamp = self.get_clock().now().to_msg()
                        message.boot_id, message.uptime_us, message.c1_count, message.c2_count = signed
                        self.signed_publisher.publish(message)
                        self.signed_received = True
                    rpm = self.speed.receive(line, time.monotonic())
                    if rpm is not None:
                        for name, value in zip(('c1_rpm','c2_rpm','rpm'), (*rpm, sum(rpm)/2)):
                            self.telemetry[name].publish(Float64(data=float(value)))
                        self.valid.publish(Bool(data=True))
                    row = self.engine.receive(line, time.monotonic())
                    if row is not None:
                        values = dict(pwm=row['pwm'], requested_rpm=row['requested'] if row['active'] else 0.0,
                                      setpoint=row['target'] if row['active'] else 0.0,
                                      position_turns=row['position_turns'])
                        for name, value in values.items():
                            self.telemetry[name].publish(Float64(data=float(value)))
                        self.state.publish(String(data=row['reason']))
                        if not self.ready and not row['active']:
                            self.ready = True
                            self.get_logger().info(f'READY: /motor/move_wheel; encoder sign={self.engine.config.sign}.')
                if len(self.buffer) > 8192:
                    self.buffer = b''
                if now-self.speed.at > 1:
                    self.valid.publish(Bool(data=False))
                if now-self.query_at >= .2:
                    self.write_line(b'LAB?\nENC?\nENC2?\n' if self.signed_enabled else b'LAB?\nENC?\n')
                    self.query_at = now
                if (self.signed_enabled and not self.signed_received and not self.signed_warned
                        and now-self.signed_started_at > 3):
                    self.get_logger().warning('No ENC2 response: flash the JointState firmware; old ENC/LAB replies do not contain idle signed position.')
                    self.signed_warned = True
            except serial.SerialException as exc:
                try:
                    self.write_line(b'MOTOR STOP\n')
                except serial.SerialException:
                    pass
                self.engine.disconnect(f'Serial error: {exc}')
                self.valid.publish(Bool(data=False))
                self.get_logger().error(f'{exc}; reconnect USB and restart server.')
                self.timer.cancel()

    def check_parameters(self, parameters):
        relevant = [p for p in parameters if p.name in PARAMETER_DEFAULTS]
        if not relevant:
            return SetParametersResult(successful=True)
        with self.lock:
            if self.engine.job is not None:
                return SetParametersResult(successful=False, reason='Motor task is busy; change configuration after its final Result.')
            candidate = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
            candidate.update({p.name:p.value for p in relevant})
            try:
                validate_parameters(candidate)
            except (ValueError,TypeError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def apply_parameters(self, parameters):
        if not any(p.name in PARAMETER_DEFAULTS for p in parameters):
            return
        with self.lock:
            values = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
            self.engine.config = validate_parameters(values)
            self.get_logger().info('Parameters saved for the next goal; motor was not started.')

    def goal(self, request):
        with self.lock:
            job, error = self.engine.reserve(request.turns, request.rpm, request.timeout_sec,
                                             time.monotonic())
        if job is None:
            self.get_logger().warning('Goal rejected: '+error)
            return GoalResponse.REJECT
        self.get_logger().info(f'Goal accepted: {job.turns:+g} turns, {job.rpm} RPM, timeout {job.timeout}s.')
        self.get_logger().info(f'PI snapshot: Kp={job.config.kp:g}, Ki={job.config.ki:g}, '
                               f'max PWM={job.config.max_pwm:g}%, antiwindup={job.config.antiwindup}.')
        return GoalResponse.ACCEPT

    def cancel(self, goal_handle):
        with self.lock:
            job = self.engine.job
            if job is None or job.done.is_set():
                return CancelResponse.REJECT
            job.cancel = True
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        with self.lock:
            job = self.engine.job
        try:
            while rclpy.ok() and not job.done.wait(.1):
                with self.lock:
                    if goal_handle.is_cancel_requested:
                        job.cancel = True
                    values = self.engine.feedback(job)
                goal_handle.publish_feedback(MoveWheel.Feedback(**values))
            with self.lock:
                current = job.row.get('position_turns', 0.0)
                result = MoveWheel.Result(success=job.outcome == 'SUCCEEDED',
                                          drive_disabled=job.disabled, final_turns=current,
                                          error_degrees=(current-job.turns)*360 if job.turns else 0.0,
                                          message=job.message or 'Server shutting down; result unconfirmed.')
            if rclpy.ok():
                if job.outcome == 'CANCELED':
                    # cancel_callback returns before rclpy marks the handle CANCELING.
                    deadline = time.monotonic()+.5
                    while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
                        time.sleep(.01)
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                elif job.outcome == 'SUCCEEDED':
                    # A late accepted cancellation wins over reporting success.
                    if goal_handle.is_cancel_requested:
                        result.success = False
                        result.message += ' Cancellation arrived after drive-off.'
                        goal_handle.canceled()
                    else:
                        goal_handle.succeed()
                else:
                    goal_handle.abort()
                self.get_logger().info(f'{job.outcome}: {result.message}')
            return result
        finally:
            with self.lock:
                self.engine.job = None

    def stop_on_shutdown(self):
        with self.lock:
            try:
                self.write_line(b'MOTOR STOP\n')
            except serial.SerialException:
                pass
            self.engine.disconnect('Server shutting down')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None)
    parser.add_argument('--encoder-sign', type=int, choices=(-1, 1), default=None)
    parser.add_argument('--tolerance-deg', type=float, default=None)
    args, ros_args = parser.parse_known_args()
    if args.tolerance_deg is not None and (not math.isfinite(args.tolerance_deg) or not 0 < args.tolerance_deg <= 180):
        parser.error('--tolerance-deg must be finite and in (0, 180]')
    rclpy.init(args=ros_args)
    # Match the standalone probe/host tools; USB passthrough can exceed 200 ms.
    port = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=2, exclusive=True)
    port.dtr = port.rts = False
    node = executor = None
    try:
        node = MotorActionServer(port, args.encoder_sign, args.tolerance_deg, args.port)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except serial.SerialException as exc:
        raise SystemExit(f'Serial error: {exc}; close the old bridge/serial monitor first.') from exc
    except (ValueError,TypeError) as exc:
        raise SystemExit(f'Invalid motor configuration: {exc}') from exc
    finally:
        if node is not None:
            node.stop_on_shutdown()
        if executor is not None:
            executor.shutdown()
        port.close()
        if node is not None:
            node.server.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
