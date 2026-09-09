#!/usr/bin/env python3
"""
Pose Filter — stabilizes noisy cube pose detections with a sliding-window
median filter. Only publishes once the window is full and stable (std below
threshold), so pick_place_server never sees a jittery pose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from collections import deque


class PoseFilter(Node):
    def __init__(self):
        super().__init__('pose_filter')

        self.declare_parameter('window_size', 10)
        self.declare_parameter('std_threshold', 0.01)

        self.window = self.get_parameter('window_size').value
        self.std_th = self.get_parameter('std_threshold').value

        self.buffer = deque(maxlen=self.window)

        self.sub = self.create_subscription(
            PoseStamped, '/urxp/cube_pose',
            self.pose_cb, 10)

        self.pub = self.create_publisher(
            PoseStamped, '/urxp/cube_pose_filtered', 10)

        self.get_logger().info('Pose Filter Node Started')

    def pose_cb(self, msg):
        self.buffer.append([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        if len(self.buffer) < self.window:
            return

        arr = np.array(self.buffer)
        std = np.std(arr, axis=0)

        if np.all(std < self.std_th):
            median = np.median(arr, axis=0)
            filtered = PoseStamped()
            filtered.header = msg.header
            filtered.pose.position.x = float(median[0])
            filtered.pose.position.y = float(median[1])
            filtered.pose.position.z = float(median[2])
            filtered.pose.orientation = msg.pose.orientation
            self.pub.publish(filtered)
        else:
            self.get_logger().warn(
                f'Pose unstable, std={std}', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PoseFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
