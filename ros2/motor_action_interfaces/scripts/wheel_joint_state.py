#!/usr/bin/env python3
"""Publish measured /joint_states; no serial access or motor commands."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from motor_action_interfaces.msg import EncoderState
from wheel_joint_core import WheelEstimator


class WheelJointState(Node):
    def __init__(self):
        super().__init__('wheel_joint_state')
        for name,value in [('encoder_sign',-1),('counts_per_rev',1400.0),('joint_name','wheel_joint')]:
            self.declare_parameter(name,value,descriptor=ParameterDescriptor(read_only=True))
        self.estimator = WheelEstimator(self.get_parameter('counts_per_rev').value,
                                        self.get_parameter('encoder_sign').value)
        self.joint_name = self.get_parameter('joint_name').value
        self.joints = self.create_publisher(JointState,'/joint_states',10)
        self.valid = self.create_publisher(Bool,'/motor/joint_valid',10)
        self.turns = self.create_publisher(Float64,'/motor/joint_turns',10)
        self.subscription = self.create_subscription(EncoderState,'/motor/encoder_signed',self.receive,10)
        self.timer = self.create_timer(.2,self.check_fresh)
        self.was_fresh = False
        self.get_logger().info('Waiting for signed encoder data. First sample defines the visual zero.')

    def receive(self, message):
        state = message.boot_id,message.uptime_us,message.c1_count,message.c2_count
        row = self.estimator.update(state,time.monotonic())
        if row is None:
            return
        joint = JointState()
        joint.header.stamp = message.header.stamp
        joint.name = [self.joint_name]
        joint.position = [row['position']]
        joint.velocity = [] if row['velocity'] is None else [row['velocity']]
        joint.effort = []  # There is no measured joint torque.
        self.joints.publish(joint)
        self.turns.publish(Float64(data=row['position']/(2*math.pi)))
        self.valid.publish(Bool(data=True))
        if row['restarted']:
            self.get_logger().warning('ESP32 restarted: reset visual zero to the new first sample.')
        if not self.was_fresh:
            self.get_logger().info('Joint feedback live; rotate the physical wheel to move the model.')
        self.was_fresh = True

    def check_fresh(self):
        fresh = self.estimator.fresh(time.monotonic())
        self.valid.publish(Bool(data=fresh))
        if self.was_fresh and not fresh:
            self.get_logger().warning('Encoder feedback stale; no new JointState. The displayed pose is last known.')
        self.was_fresh = fresh


def main():
    rclpy.init()
    node = None
    try:
        node = WheelJointState()
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
