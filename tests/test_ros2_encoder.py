"""Exercise actual ROS bridge code with fake ROS interfaces; no serial hardware needed."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.publishers = {}
        self.logger = Mock()

    def create_publisher(self, message_type, topic, depth):
        publisher = SimpleNamespace(messages=[])
        publisher.publish = publisher.messages.append
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, message_type, topic, callback, depth):
        return SimpleNamespace(topic=topic, callback=callback, depth=depth)

    def create_service(self, service_type, name, callback):
        return SimpleNamespace(service_type=service_type, name=name, callback=callback)

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        pass


class Message:
    def __init__(self, *, data):
        self.data = data


def bridge_module():
    stubs = {name: ModuleType(name) for name in ('rclpy', 'rclpy.node', 'rclpy.executors', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv')}
    stubs['rclpy.node'].Node = FakeNode
    stubs['rclpy.executors'].ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
    for name in ('Bool', 'Float64', 'String', 'UInt32MultiArray'):
        setattr(stubs['std_msgs.msg'], name, Message)
    stubs['std_srvs.srv'].Trigger = type('Trigger', (), {})
    spec = importlib.util.spec_from_file_location('encoder_bridge_under_test',
        Path(__file__).resolve().parents[1] / 'ros2/encoder_publisher.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


bridge = bridge_module()


class EstimatorTests(unittest.TestCase):
    def test_known_speed_independent_channels_and_stop(self):
        speed = bridge.RpmEstimator()
        self.assertIsNone(speed.update(0, [0, 0]))
        for tick in range(1, 11):
            rpm = speed.update(tick * .2, [280 * tick, 140 * tick])
            self.assertAlmostEqual(rpm[0], 60)
            self.assertAlmostEqual(rpm[1], 30)
        for tick in range(11, 17):
            rpm = speed.update(tick * .2, [2800, 1400])
        self.assertEqual(rpm, (0, 0))

    def test_reboot_gap_and_clock_reset_do_not_spike(self):
        for timestamp, counts in [(1.4, [0, 0]), (4.0, [9999, 9999]), (.1, [1300, 1300])]:
            with self.subTest(timestamp=timestamp):
                speed = bridge.RpmEstimator()
                speed.update(1, [1000, 1000])
                self.assertAlmostEqual(speed.update(1.2, [1280, 1280])[0], 60)
                self.assertIsNone(speed.update(timestamp, counts))
                self.assertAlmostEqual(speed.update(timestamp+.2, [x+280 for x in counts])[0],60)

    def test_true_uint32_wrap(self):
        speed = bridge.RpmEstimator()
        speed.update(1, [2**32-100, 2**32-100])
        rpm = speed.update(1.2, [180, 180])
        self.assertAlmostEqual(rpm[0], 60)


class FakePort:
    def __init__(self):
        self.port = '/dev/fake'
        self.data = b''
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, length):
        result, self.data = self.data[:length], self.data[length:]
        return result

    def write(self, data):
        self.writes.append(data)
        return len(data)


class PublisherTests(unittest.TestCase):
    def test_fragmented_responses_rpm_timeout_and_readonly_commands(self):
        port = FakePort()
        node = bridge.EncoderPublisher(port)
        def poll(timestamp, data=b''):
            port.data += data
            with patch.object(bridge.time, 'monotonic', return_value=timestamp):
                node.poll()
        poll(1)
        self.assertEqual(port.writes, [b'ENC?\n'])
        poll(1.02, b'ENC 0 0')
        self.assertFalse(node.publisher.messages)
        poll(1.04, b' 1 0\n')
        self.assertEqual(node.publisher.messages[-1].data, [0, 0])
        self.assertFalse(node.rpm_valid.messages[-1].data)
        self.assertFalse(node.rpm.messages)
        poll(1.24, b'ENC 280 140 0 1\nACK MOTOR\n')
        self.assertAlmostEqual(node.rpm.messages[-1].data,45)
        self.assertTrue(node.rpm_valid.messages[-1].data)
        before = len(node.rpm.messages)
        poll(2.5)
        self.assertFalse(node.rpm_valid.messages[-1].data)
        self.assertEqual(len(node.rpm.messages), before)  # No fake zero on a lost connection.
        poll(2.6,b'ENC 0 0 1 1\n')
        self.assertFalse(node.rpm_valid.messages[-1].data)
        self.assertEqual(len(node.rpm.messages), before)
        self.assertTrue(all(command == b'ENC?\n' for command in port.writes))

    def test_malformed_lines_are_ignored(self):
        self.assertEqual(bridge.parse_counts(b'ENC 0 0 1 0'), [0, 0])
        for line in (b'ENC -1 0 1 0', b'ENC 0 0 2 0', b'ENC 0 x 1 0',
                     b'ENC 4294967296 0 1 0', b'ACK MOTOR', b'ENC 1 0 1'):
            self.assertIsNone(bridge.parse_counts(line))


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(bridge.time, 'monotonic', return_value=10.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.writes, self.goals, self.states = [], [], []
        self.control = bridge.MotorCommands(self.writes.append, Mock(),
                                            self.goals.append, self.states.append)

    def status(self, active=False, target=120, reason='IDLE', seconds=5, sign=-1):
        return dict(active=active, target=target, reason=reason, seconds=seconds, sign=sign,
                    pwm=25 if active else 0, elapsed=0)

    def ready(self):
        self.control.encoder_at = self.now.return_value
        self.control.handle_status(self.status())

    def start(self):
        self.ready()
        self.control.receive(60.0)
        self.assertEqual(self.writes, [b'MOTOR PI 60 5 FWD -1\n'])

    def test_readiness_and_validation_never_start_motion(self):
        self.control.receive(60)
        self.assertFalse(self.writes)
        self.ready()
        for value in (float('nan'), float('inf'), -60, 39, 181, 60.5):
            self.control.receive(value)
        self.assertFalse(self.writes)
        self.now.return_value = 12
        self.control.receive(60)
        self.assertFalse(self.writes)

    def test_busy_board_is_not_overridden(self):
        self.ready()
        self.control.handle_status(self.status(active=True))
        self.control.receive(60)
        self.assertFalse(self.writes)

    def test_one_shot_duplicate_done_and_explicit_rearm(self):
        self.start()
        started_at = self.control.started_at
        self.control.receive(60)
        self.control.receive(120)
        self.assertEqual(len(self.writes), 1)
        self.assertEqual(self.control.started_at, started_at)
        # In-flight old idle reply does not falsely report completion.
        self.control.handle_status(self.status())
        self.assertTrue(self.control.active)
        self.control.handle_status(self.status(True, 60, 'WAITING'))
        self.assertTrue(self.control.confirmed)
        self.control.handle_status(self.status(False, 60, 'DONE'))
        self.assertFalse(self.control.active)
        self.assertEqual(self.goals[-1], 0)
        self.control.receive(60)
        self.assertEqual(len(self.writes), 1)
        self.control.receive(0)
        self.assertEqual(self.writes[-1], b'MOTOR STOP\n')
        self.control.receive(60)
        self.assertEqual(self.writes[-1], b'MOTOR PI 60 5 FWD -1\n')

    def test_missing_feedback_confirmation_and_host_deadline_stop(self):
        for mode in ('feedback', 'confirmation', 'deadline'):
            with self.subTest(mode=mode):
                self.control.active = False
                self.control.armed = True
                self.now.return_value = 10
                self.writes.clear()
                self.start()
                self.now.return_value = 11.1 if mode != 'deadline' else 19.1
                if mode != 'feedback':
                    self.control.encoder_at = self.control.status_at = self.now.return_value
                if mode == 'deadline':
                    self.control.confirmed = True
                self.control.tick()
                self.assertEqual(self.writes[-1], b'MOTOR STOP\n')
                self.assertFalse(self.control.active)
                expected = dict(feedback='FEEDBACK_TIMEOUT',confirmation='START_NOT_CONFIRMED',deadline='HOST_TIMEOUT')
                self.assertEqual(self.states[-1], expected[mode])

    def test_mismatched_board_parameters_stop(self):
        self.start()
        self.control.handle_status(self.status(True, 120, 'RUNNING'))
        self.assertEqual(self.writes[-1], b'MOTOR STOP\n')
        self.assertEqual(self.states[-1], 'PARAMETER_MISMATCH')

    def test_control_main_sends_stop_on_ctrl_c_and_closes_port(self):
        port = FakePort()
        port.is_open = False
        port.open = lambda: setattr(port, 'is_open', True)
        port.close = lambda: setattr(port, 'is_open', False)
        port.reset_input_buffer = lambda: None
        with patch.object(sys, 'argv', ['encoder_publisher.py', '--enable-control']), \
             patch.object(bridge.serial, 'Serial', return_value=port), \
             patch.object(bridge.rclpy, 'init', create=True), \
             patch.object(bridge.rclpy, 'spin', side_effect=KeyboardInterrupt, create=True), \
             patch.object(bridge.rclpy, 'ok', return_value=False, create=True):
            bridge.main()
        self.assertEqual(port.writes, [b'MOTOR STOP\n'])
        self.assertFalse(port.is_open)

    def test_parser_rejects_invalid_or_nonfinite_status(self):
        self.assertIsNotNone(bridge.parse_motor_status(b'PI3 1 60 0 0 25 0 WAITING 5 -1'))
        for line in (b'PI3 1 nan 0 0 25 0 WAITING 5 -1', b'PI3 2 60 0 0 25 0 RUNNING 5 -1',
                     b'PI3 1 60 0 0 101 0 RUNNING 5 -1', b'PI2 0 120 0 0 0 0 IDLE 15',
                     b'PI3 1 60 0 0 25 0 RUNNING 0 -1'):
            self.assertIsNone(bridge.parse_motor_status(line))

    def test_complete_node_queries_once_and_publishes_feedback(self):
        port = FakePort()
        node = bridge.EncoderPublisher(port, enable_control=True)
        node.poll()
        self.assertEqual(port.writes, [b'ENC?\n', b'PI?\n'])
        self.assertFalse(node.commands.active)
        port.data = b'ENC 0 0 1 0\nPI3 0 120 0 0 0 0 IDLE 15 -1\n'
        node.poll()
        node.on_target(Message(data=60.0))
        self.assertEqual(port.writes[-1], b'MOTOR PI 60 5 FWD -1\n')
        port.data = b'ACK MOTOR\nPI3 1 60 0 0 25 0 WAITING 5 -1\n'
        node.poll()
        self.assertTrue(node.commands.confirmed)
        self.assertEqual(node.state.messages[-1].data, 'WAITING')
        self.assertEqual(node.setpoint.messages[-1].data, 60)
        self.assertEqual(node.pwm.messages[-1].data, 25)
        node.on_target(Message(data=0.0))
        self.assertEqual(port.writes[-1], b'MOTOR STOP\n')
        self.assertFalse(node.commands.active)


class StopServiceTests(unittest.TestCase):
    def setUp(self):
        self.port = FakePort()
        self.node = bridge.EncoderPublisher(self.port, enable_control=True)
        self.response = SimpleNamespace(success=False, message='')
        self.clock = patch.object(bridge.time, 'monotonic', return_value=10.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)

    def call_stop(self):
        return self.node.on_stop(SimpleNamespace(), self.response)

    def test_success_requires_ack_then_fresh_inactive_zero_pwm(self):
        # Reply delivery here is driven by the fake board receiving the STOP and PI?.
        original_write = self.port.write
        def write(command):
            result = original_write(command)
            if command == b'MOTOR STOP\n':
                self.port.data += b'ACK MOTOR\n'
            elif command == b'PI?\n':
                self.port.data += b'PI3 0 60 40 40 0 2 STOPPED 5 -1\n'
            return result
        self.port.write = write
        response = self.call_stop()
        self.assertTrue(response.success)
        self.assertIn('Wheel may coast', response.message)
        self.assertEqual(self.port.writes[0], b'MOTOR STOP\n')
        self.assertFalse(self.node.commands.armed)
        self.assertFalse(self.node.stop_in_progress)
        self.assertFalse(any(b'MOTOR PI ' in c for c in self.port.writes))

    def test_cached_status_and_old_ack_cannot_confirm_stop(self):
        # An earlier command's ACK plus an old stopped reply arrive before our new ACK.
        self.node.motor_commands_sent = 1
        self.port.data = b'ACK MOTOR\nPI3 0 120 0 0 0 0 DONE 15 -1\n'
        reads = [10.0]
        def monotonic():
            reads[0] += .05
            return reads[0]
        self.now.side_effect = monotonic
        with patch.object(bridge.time, 'sleep'):
            response = self.call_stop()
        self.assertFalse(response.success)
        self.assertEqual(self.node.motor_commands_sent, 2)
        self.assertEqual(self.node.motor_commands_acked, 1)
        self.assertIn('1.5s', response.message)

    def test_status_before_ack_is_not_fresh_confirmation(self):
        self.port.data = b'PI3 0 60 0 0 0 0 STOPPED 5 -1\nACK MOTOR\n'
        values = [10.0]
        def clock():
            values[0] += .05
            return values[0]
        self.now.side_effect = clock
        with patch.object(bridge.time, 'sleep'):
            response = self.call_stop()
        self.assertFalse(response.success)
        self.assertEqual(self.node.motor_commands_acked, 1)
        self.assertEqual(self.node.status_ack_sequence, 0)

    def test_active_or_nonzero_pwm_is_not_success(self):
        for status in (b'PI3 1 60 60 60 25 1 RUNNING 5 -1\n',
                       b'PI3 0 60 0 0 25 1 STOPPED 5 -1\n'):
            with self.subTest(status=status):
                self.port = FakePort()
                self.node = bridge.EncoderPublisher(self.port, enable_control=True)
                self.response = SimpleNamespace(success=False, message='')
                self.port.data = b'ACK MOTOR\n' + status
                value = [10.0]
                def clock():
                    value[0] += .05
                    return value[0]
                self.now.side_effect = clock
                with patch.object(bridge.time, 'sleep'):
                    self.assertFalse(self.call_stop().success)

    def test_serial_failure_returns_false_and_releases_guard(self):
        self.port.write = Mock(side_effect=bridge.serial.SerialException('disconnected'))
        response = self.call_stop()
        self.assertFalse(response.success)
        self.assertIn('disconnected', response.message)
        self.assertFalse(self.node.stop_in_progress)
        self.assertFalse(self.node.commands.armed)

    def test_topic_commands_cannot_restart_during_stop(self):
        self.node.stop_in_progress = True
        self.node.on_target(Message(data=60.0))
        self.assertFalse(self.port.writes)
        self.assertEqual(self.node.stop_service.name, '/motor/stop')


if __name__ == '__main__':
    unittest.main()
