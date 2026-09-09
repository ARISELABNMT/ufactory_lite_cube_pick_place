#!/usr/bin/env python3
"""
Gripper control node for xArm Lite6 vacuum gripper via Tool GPIO.

Runs as a separate process to avoid xArm SDK dual-connection error C19.
Subscribes to /urxp/gripper/command (True=open, False=close).
Publishes /urxp/gripper/state (OPEN / CLOSED / BUSY).

Tool GPIO wiring (actual hardware, verified by observation):
  pin0 = 1 -> close (suck/grip)
  pin1 = 1 -> open (blow/release)
A keepalive thread pulses pin0 every 20 ms while closed to maintain grip.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class GripperNode(Node):
    def __init__(self):
        super().__init__('urxp_gripper_node')

        self.declare_parameter('robot_ip', '192.168.1.165')
        robot_ip = self.get_parameter('robot_ip').value

        from xarm.wrapper import XArmAPI
        self._arm = XArmAPI(robot_ip, baud_checkset=False)
        self._keep_closed = False
        self._lock = threading.Lock()

        self.create_subscription(Bool, '/urxp/gripper/command', self._on_command, 10)
        self._state_pub = self.create_publisher(String, '/urxp/gripper/state', 10)

        self._keepalive_thread = threading.Thread(
            target=self._keepalive, daemon=True)
        self._keepalive_thread.start()

        self.get_logger().info(f'Gripper node ready (IP={robot_ip})')

    def _on_command(self, msg: Bool):
        if msg.data:
            self._open()
        else:
            self._close()

    def _open(self):
        self._publish_state('BUSY')
        with self._lock:
            self._keep_closed = False  # stop keepalive from pulsing
        time.sleep(0.05)  # let any in-flight keepalive hardware pulse expire
        with self._lock:
            self._arm.set_tgpio_digital(0, 0)  # cancel close (pin0 off)
            self._arm.set_tgpio_digital(1, 1)  # open/blow (pin1 on)
        time.sleep(1.5)
        with self._lock:
            self._arm.set_tgpio_digital(1, 0)  # stop open pulse (pin1 off)
        self._publish_state('OPEN')
        self.get_logger().info('Gripper OPEN')

    def _close(self):
        self._publish_state('BUSY')
        with self._lock:
            self._arm.set_tgpio_digital(1, 0)  # cancel open (pin1 off)
            self._arm.set_tgpio_digital(0, 1)  # close/suck (pin0 on)
            self._keep_closed = True
        time.sleep(1.5)
        self._publish_state('CLOSED')
        self.get_logger().info('Gripper CLOSED')

    def _keepalive(self):
        while True:
            with self._lock:
                if self._keep_closed:
                    self._arm.set_tgpio_digital(0, 1)  # maintain suck (pin0 on)
            time.sleep(0.02)

    def _publish_state(self, state: str):
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)


def main():
    rclpy.init()
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
