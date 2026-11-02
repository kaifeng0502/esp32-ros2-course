"""Exercise actual Jazzy parameter callbacks with ROS/serial stand-ins."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from test_ros2_action import core, status

wheel_spec = importlib.util.spec_from_file_location('wheel_joint_core',
    Path(__file__).resolve().parents[1]/'ros2/motor_action_interfaces/scripts/wheel_joint_core.py')
wheel_core = importlib.util.module_from_spec(wheel_spec)
wheel_spec.loader.exec_module(wheel_core)


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.logger = Mock()

    def declare_parameter(self,name,value,descriptor=None):
        self.parameters[name] = SimpleNamespace(name=name,value=self.overrides.get(name,value))

    def get_parameter(self,name):
        return self.parameters[name]

    def add_on_set_parameters_callback(self,callback):
        self.check = callback

    def add_post_set_parameters_callback(self,callback):
        self.post = callback

    def set_parameters_for_test(self,**changes):
        parameters = [SimpleNamespace(name=k,value=v) for k,v in changes.items()]
        result = self.check(parameters)
        if result.successful:
            self.parameters.update({p.name:p for p in parameters})
            self.post(parameters)
        return result

    def create_publisher(self,*args):
        return Mock()

    def create_timer(self,*args):
        return Mock()

    def get_logger(self):
        return self.logger


def load_server():
    names = ('rclpy','rclpy.action','rclpy.callback_groups','rclpy.executors','rclpy.node',
             'rcl_interfaces','rcl_interfaces.msg','std_msgs','std_msgs.msg',
             'motor_action_interfaces','motor_action_interfaces.action','motor_action_interfaces.msg')
    stubs = {name:ModuleType(name) for name in names}
    stubs['rclpy.node'].Node = FakeNode
    for name in ('ActionServer','CancelResponse','GoalResponse'):
        setattr(stubs['rclpy.action'],name,Mock())
    stubs['rclpy.callback_groups'].ReentrantCallbackGroup = Mock()
    stubs['rclpy.executors'].ExternalShutdownException = type('ExternalShutdownException',(Exception,),{})
    stubs['rclpy.executors'].MultiThreadedExecutor = Mock()
    for name in ('ParameterDescriptor','SetParametersResult'):
        setattr(stubs['rcl_interfaces.msg'],name,SimpleNamespace)
    for name in ('Bool','Float64','String'):
        setattr(stubs['std_msgs.msg'],name,SimpleNamespace)
    stubs['motor_action_interfaces.action'].MoveWheel = Mock()
    stubs['motor_action_interfaces.msg'].EncoderState = Mock()
    stubs['motor_action_core'] = core
    stubs['wheel_joint_core'] = wheel_core
    source = Path(__file__).resolve().parents[1]/'ros2/motor_action_interfaces/scripts/motor_action_server.py'
    spec = importlib.util.spec_from_file_location('action_server_parameter_test',source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,stubs):
        spec.loader.exec_module(module)
    return module


server = load_server()


class ParameterTests(unittest.TestCase):
    def setUp(self):
        FakeNode.overrides = {}
        self.port = Mock()
        self.port.write.side_effect = len

    def test_startup_overrides_and_no_motor_command(self):
        FakeNode.overrides = dict(kp=.25,ki=.7,port='/dev/ttyACM1')
        node = server.MotorActionServer(self.port)
        self.assertEqual(node.engine.config.kp,.25)
        self.assertEqual(self.port.port,'/dev/ttyACM1')
        self.port.open.assert_called_once()
        self.port.write.assert_not_called()

    def test_parameter_set_only_changes_next_goal_configuration(self):
        node = server.MotorActionServer(self.port)
        result = node.set_parameters_for_test(kp=.2,ki=.6,target_rpm=80,run_seconds=5)
        self.assertTrue(result.successful)
        self.assertEqual(node.engine.config.ki,.6)
        self.port.write.assert_not_called()
        node.engine.receive(status(),0)
        job,_ = node.engine.reserve(0.0,80,5,.1)
        node.engine.tick(.1)
        self.assertIn(b'MOTOR LAB 80 5 FWD -1 PI 0 0.2 0.6',self.port.write.call_args_list[0].args[0])
        old = node.engine.config
        result = node.set_parameters_for_test(kp=.7)
        self.assertFalse(result.successful)
        self.assertEqual(node.engine.config,old)
        self.assertEqual(job.config.kp,.2)

    def test_invalid_parameters_rejected_without_writes(self):
        node = server.MotorActionServer(self.port)
        for changes in [dict(target_rpm=500),dict(run_seconds=0),dict(kp=-1.0),
                        dict(ki=float('nan')),dict(max_pwm=90.0),dict(encoder_sign=0),
                        dict(target_turns=.001),dict(target_rpm=True),dict(antiwindup=1)]:
            with self.subTest(changes=changes):
                self.assertFalse(node.set_parameters_for_test(**changes).successful)
        self.assertEqual(node.get_parameter('target_rpm').value,120)
        self.port.write.assert_not_called()

    def test_invalid_startup_fails_before_opening_port(self):
        FakeNode.overrides = dict(kp=-1.0)
        with self.assertRaises(ValueError):
            server.MotorActionServer(self.port)
        self.port.open.assert_not_called()

    def test_signed_encoder_forwarding_uses_original_counts_and_ros_stamp(self):
        FakeNode.overrides = dict(publish_signed_encoder=True)
        node = server.MotorActionServer(self.port)
        node.get_clock = Mock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = 'ROS stamp'
        self.port.in_waiting = 30
        self.port.read.return_value = b'ENC2 8 200000 -123 -125\n'
        node.poll()
        row = node.signed_publisher.publish.call_args.args[0]
        self.assertEqual((row.boot_id,row.uptime_us,row.c1_count,row.c2_count),(8,200000,-123,-125))
        self.assertEqual(row.header.stamp,'ROS stamp')
        self.assertTrue(node.signed_received)
        self.assertIn(b'ENC2?',self.port.write.call_args.args[0])

    def test_legacy_mode_does_not_query_or_publish_new_signed_interface(self):
        node = server.MotorActionServer(self.port)
        self.port.in_waiting = 0
        self.port.read.return_value = b''
        node.poll()
        self.assertNotIn(b'ENC2?',self.port.write.call_args.args[0])
        node.signed_publisher.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
