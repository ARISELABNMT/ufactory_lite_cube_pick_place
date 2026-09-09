#!/usr/bin/env python3
"""
Simulated vacuum gripper node for Gazebo.

Same interface as the real gripper_node.py (/urxp/gripper/command in,
/urxp/gripper/state out) so pick_place_server is hardware-agnostic. Instead
of Tool GPIO, it triggers the DetachableJoint plugin attached to the target
cube's URDF link (see xarm_description's lite_vacuum_gripper.urdf.xacro) by
publishing on the two gz-bridged topics /urxp/gripper_attach and
/urxp/gripper_detach — bridged to Gazebo by urxp_gazebo.launch.py.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Empty


class GripperSimNode(Node):
    def __init__(self):
        super().__init__('urxp_gripper_sim_node')

        self.create_subscription(Bool, '/urxp/gripper/command', self._on_command, 10)
        self._state_pub = self.create_publisher(String, '/urxp/gripper/state', 10)
        self._attach_pub = self.create_publisher(Empty, '/urxp/gripper_attach', 10)
        self._detach_pub = self.create_publisher(Empty, '/urxp/gripper_detach', 10)

        self.get_logger().info('Gripper sim node ready (Gazebo DetachableJoint)')

    def _on_command(self, msg: Bool):
        if msg.data:
            self._open()
        else:
            self._close()

    def _open(self):
        self._publish_state('BUSY')
        self._detach_pub.publish(Empty())
        time.sleep(0.3)
        self._publish_state('OPEN')
        self.get_logger().info('Gripper OPEN (detached)')

    def _close(self):
        self._publish_state('BUSY')
        self._attach_pub.publish(Empty())
        time.sleep(0.3)
        self._publish_state('CLOSED')
        self.get_logger().info('Gripper CLOSED (attached)')

    def _publish_state(self, state: str):
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)


def main():
    rclpy.init()
    node = GripperSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
