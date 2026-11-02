"""Actual JointState callbacks with ROS stand-ins; no motors or serial devices."""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType,SimpleNamespace
import unittest
from unittest.mock import patch
from test_ros2_parameters import FakeNode, wheel_core


class JointNode(FakeNode):
    def create_subscription(self,*args):
        return SimpleNamespace()


class JointMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)


names = ('rclpy','rclpy.node','rclpy.executors','rcl_interfaces','rcl_interfaces.msg',
         'sensor_msgs','sensor_msgs.msg','std_msgs','std_msgs.msg',
         'motor_action_interfaces','motor_action_interfaces.msg')
stubs = {name:ModuleType(name) for name in names}
stubs['rclpy.node'].Node = JointNode
stubs['rclpy.executors'].ExternalShutdownException = type('ExternalShutdownException',(Exception,),{})
stubs['rcl_interfaces.msg'].ParameterDescriptor = SimpleNamespace
stubs['sensor_msgs.msg'].JointState = JointMessage
stubs['std_msgs.msg'].Bool = SimpleNamespace
stubs['std_msgs.msg'].Float64 = SimpleNamespace
stubs['motor_action_interfaces.msg'].EncoderState = SimpleNamespace
stubs['wheel_joint_core'] = wheel_core
spec = importlib.util.spec_from_file_location('wheel_node_under_test',
    Path(__file__).resolve().parents[1]/'ros2/motor_action_interfaces/scripts/wheel_joint_state.py')
wheel_node = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules,stubs):
    spec.loader.exec_module(wheel_node)


class JointNodeTests(unittest.TestCase):
    def test_measured_radians_timestamp_and_no_invented_torque(self):
        FakeNode.overrides = {}
        node = wheel_node.WheelJointState()
        def message(us,count):
            return SimpleNamespace(boot_id=1,uptime_us=us,c1_count=count,c2_count=count,
                                   header=SimpleNamespace(stamp='original ROS timestamp'))
        with patch.object(wheel_node.time,'monotonic',return_value=0):
            node.receive(message(0,0))
        first = node.joints.publish.call_args.args[0]
        self.assertEqual(first.velocity,[])
        with patch.object(wheel_node.time,'monotonic',return_value=.2):
            node.receive(message(200000,-1400))
        row = node.joints.publish.call_args.args[0]
        self.assertEqual(row.name,['wheel_joint'])
        self.assertAlmostEqual(row.position[0],2*math.pi)
        self.assertGreater(row.velocity[0],0)
        self.assertEqual(row.header.stamp,'original ROS timestamp')
        self.assertEqual(row.effort,[])
        count = node.joints.publish.call_count
        with patch.object(wheel_node.time,'monotonic',return_value=2):
            node.check_fresh()
        self.assertFalse(node.valid.publish.call_args.args[0].data)
        self.assertEqual(node.joints.publish.call_count,count)


if __name__ == '__main__':
    unittest.main()
