#!/usr/bin/env python3
"""
Cube Detector Node — Color + Contour based detection.
Publishes the 3D pose of a detected colored cube in the robot base frame.
"""

import math

import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)
from image_geometry import PinholeCameraModel


def _yaw_quaternion(raw_angle_deg: float) -> Quaternion:
    """Fold cv2.minAreaRect's angle to [-45, 45] deg by cube symmetry and
    return a proper quaternion for a top-down marker rotated by that yaw
    (matches pick_place_server's grasp-orientation convention)."""
    folded = raw_angle_deg % 90.0
    if folded > 45.0:
        folded -= 90.0
    yaw = math.radians(folded)
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('cube_color', 'multi')
        self.declare_parameter('min_area', 80)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('robot_base_frame', 'link_base')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('fallback_depth_m', 0.40)
        self.declare_parameter('min_solidity', 0.75)
        self.declare_parameter('morph_kernel_size', 5)
        # Workspace ROI as fractions of image width/height [0,1] — excludes
        # background clutter (other objects, cables, edges of the desk) from
        # ever being considered, regardless of color. Defaults to full frame;
        # set tighter bounds in detector_params.yaml for your actual bench.
        self.declare_parameter('roi_x_min', 0.0)
        self.declare_parameter('roi_x_max', 1.0)
        self.declare_parameter('roi_y_min', 0.0)
        self.declare_parameter('roi_y_max', 1.0)
        # Topic names differ between the real RealSense driver and the Gazebo
        # sim bridge — overridden per-launch-file rather than hardcoded.
        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('color_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('depth_image_topic', '/camera/camera/aligned_depth_to_color/image_raw')

        self.cube_color        = self.get_parameter('cube_color').value
        self.min_area          = self.get_parameter('min_area').value
        self.max_area          = self.get_parameter('max_area').value
        self.base_frame        = self.get_parameter('robot_base_frame').value
        self.cam_frame          = self.get_parameter('camera_frame').value
        self.fallback_depth_m  = self.get_parameter('fallback_depth_m').value
        self.min_solidity      = self.get_parameter('min_solidity').value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        self.roi_x_min         = self.get_parameter('roi_x_min').value
        self.roi_x_max         = self.get_parameter('roi_x_max').value
        self.roi_y_min         = self.get_parameter('roi_y_min').value
        self.roi_y_max         = self.get_parameter('roi_y_max').value
        color_image_topic      = self.get_parameter('color_image_topic').value
        color_info_topic       = self.get_parameter('color_info_topic').value
        depth_image_topic      = self.get_parameter('depth_image_topic').value

        # ── Internal state ───────────────────────────────────────
        self.bridge       = CvBridge()
        self.cam_model    = PinholeCameraModel()
        self.cam_info_rcv = False
        self.depth_image  = None

        # ── TF2 ─────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            CameraInfo, color_info_topic,
            self.camera_info_cb, 10)
        self.create_subscription(
            Image, color_image_topic,
            self.color_image_cb, 10)
        self.create_subscription(
            Image, depth_image_topic,
            self.depth_image_cb, 10)

        # ── Publishers ───────────────────────────────────────────
        self.pose_pub  = self.create_publisher(PoseStamped, '/urxp/cube_pose', 10)
        self.color_pub = self.create_publisher(String, '/urxp/cube_color', 10)
        self.debug_pub = self.create_publisher(Image, '/urxp/debug_image', 10)
        self.marker_pub = self.create_publisher(Marker, '/urxp/marker', 10)

        self.get_logger().info('Cube Detector Node Started')

    # ────────────────────────────────────────────────────────────
    def camera_info_cb(self, msg):
        self.cam_model.from_camera_info(msg)
        self.cam_info_rcv = True

    def depth_image_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    # ────────────────────────────────────────────────────────────
    def color_image_cb(self, msg):
        if not self.cam_info_rcv:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        debug_img = cv_image.copy()

        bbox, center_px, angle_deg, detected_color = self.detect_cube(cv_image)

        if center_px is None:
            self.get_logger().warn('No cube detected', throttle_duration_sec=2.0)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
            self.debug_pub.publish(debug_msg)
            return

        u, v = int(center_px[0]), int(center_px[1])
        depth_m = self.get_depth(u, v)

        if depth_m is None or depth_m <= 0.0:
            depth_m = self.fallback_depth_m
            self.get_logger().warn(
                f'Invalid depth — using fallback {depth_m:.3f}m',
                throttle_duration_sec=2.0)

        point_camera = self.pixel_to_3d(u, v, depth_m)
        point_base = self.transform_to_base(point_camera, msg.header.stamp)

        if point_base is None:
            return

        self.publish_pose(point_base, msg.header.stamp, angle_deg)

        color_msg = String()
        color_msg.data = detected_color
        self.color_pub.publish(color_msg)

        if bbox is not None:
            cv2.drawContours(debug_img, [bbox], 0, (0, 255, 0), 2)
            cv2.circle(debug_img, (u, v), 5, (0, 0, 255), -1)
            cv2.putText(debug_img,
                        f'{detected_color} {depth_m:.3f}m',
                        (bbox[0][0], bbox[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
        self.debug_pub.publish(debug_msg)

    # ────────────────────────────────────────────────────────────
    def detect_cube(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_ranges = {
            'white':  ([0,   0,   180], [180, 40,  255]),
            'red':    ([0,   120, 70],  [10,  255, 255]),
            'blue':   ([100, 120, 70],  [130, 255, 255]),
            'green':  ([40,  50,  50],  [80,  255, 255]),
            'black':  ([0,   0,   0],   [180, 255, 50]),
            'yellow': ([20,  100, 100], [35,  255, 255]),
        }

        if self.cube_color == 'multi':
            # Narrow, non-overlapping HSV ranges per color.
            # Red wraps around in HSV — needs two ranges.
            mask_r1 = cv2.inRange(hsv, np.array([0,   80, 50]),  np.array([15,  255, 255]))
            mask_r2 = cv2.inRange(hsv, np.array([160, 80, 50]),  np.array([180, 255, 255]))
            mask_r  = cv2.bitwise_or(mask_r1, mask_r2)
            mask_g  = cv2.inRange(hsv, np.array([40,  80, 50]),  np.array([85,  255, 255]))
            mask_b  = cv2.inRange(hsv, np.array([95,  80, 50]),  np.array([135, 255, 255]))
            masks = {'red': mask_r, 'green': mask_g, 'blue': mask_b}
            mask = cv2.bitwise_or(mask_r, cv2.bitwise_or(mask_g, mask_b))
        else:
            lower, upper = color_ranges.get(
                self.cube_color, color_ranges['white'])
            mask = cv2.inRange(
                hsv, np.array(lower), np.array(upper))
            masks = {self.cube_color: mask}

        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Workspace ROI: zero out anything outside the configured region so
        # background clutter (other objects, cables, desk edges) is never a
        # candidate, regardless of color match.
        h_img, w_img = mask.shape[:2]
        x1r = int(self.roi_x_min * w_img)
        x2r = int(self.roi_x_max * w_img)
        y1r = int(self.roi_y_min * h_img)
        y2r = int(self.roi_y_max * h_img)
        if (x1r, x2r, y1r, y2r) != (0, w_img, 0, h_img):
            roi_mask = np.zeros_like(mask)
            roi_mask[y1r:y2r, x1r:x2r] = 255
            mask = cv2.bitwise_and(mask, roi_mask)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, None, None, 'unknown'

        best = None
        best_score = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue

            # Reject irregular/jagged blobs (clutter, shadows, fringing artifacts).
            # A real cube face is a clean, convex quadrilateral — solidity near 1.0.
            hull_area = cv2.contourArea(cv2.convexHull(cnt))
            solidity = area / hull_area if hull_area > 0 else 0.0
            if solidity < self.min_solidity:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect = float(w) / h if h > 0 else 0
            squareness = 1.0 - abs(1.0 - aspect)
            score = area * squareness

            if score > best_score:
                best_score = score
                best = cnt

        if best is None:
            return None, None, None, 'unknown'

        # Determine color by counting matching pixels inside the cube bounding box.
        bx, by, bw, bh = cv2.boundingRect(best)
        y1 = max(0, by)
        y2 = min(image.shape[0], by + bh)
        x1 = max(0, bx)
        x2 = min(image.shape[1], bx + bw)
        detected_color = 'unknown'
        best_color_score = 0
        for color_name, color_mask in masks.items():
            score = cv2.countNonZero(color_mask[y1:y2, x1:x2])
            if score > best_color_score:
                best_color_score = score
                detected_color = color_name

        rect = cv2.minAreaRect(best)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        cx, cy = int(rect[0][0]), int(rect[0][1])
        angle_deg = rect[2]  # rotation angle of bounding box in image plane

        return box, (cx, cy), angle_deg, detected_color

    # ────────────────────────────────────────────────────────────
    def get_depth(self, u, v, window=5):
        if self.depth_image is None:
            return None
        h, w = self.depth_image.shape
        u1 = max(0, u - window)
        u2 = min(w, u + window)
        v1 = max(0, v - window)
        v2 = min(h, v + window)

        patch = self.depth_image[v1:v2, u1:u2].astype(np.float32)
        patch[patch == 0] = np.nan
        median = np.nanmedian(patch)

        if np.isnan(median):
            return None

        return float(median) / 1000.0

    # ────────────────────────────────────────────────────────────
    def pixel_to_3d(self, u, v, depth_m):
        fx = self.cam_model.fx()
        fy = self.cam_model.fy()
        cx = self.cam_model.cx()
        cy = self.cam_model.cy()

        X = (u - cx) * depth_m / fx
        Y = (v - cy) * depth_m / fy
        Z = depth_m

        return np.array([X, Y, Z])

    # ────────────────────────────────────────────────────────────
    def transform_to_base(self, point_camera, stamp):
        point_stamped = PointStamped()
        point_stamped.header.stamp = rclpy.time.Time().to_msg()
        point_stamped.header.frame_id = self.cam_frame
        point_stamped.point.x = float(point_camera[0])
        point_stamped.point.y = float(point_camera[1])
        point_stamped.point.z = float(point_camera[2])

        try:
            transformed = self.tf_buffer.transform(
                point_stamped, self.base_frame,
                timeout=rclpy.duration.Duration(seconds=1.0))
            return transformed
        except Exception as e:
            self.get_logger().error(f'TF transform failed: {e}')
            return None

    # ────────────────────────────────────────────────────────────
    def publish_pose(self, point_base, stamp, angle_deg=0.0):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame

        pose.pose.position.x = point_base.point.x
        pose.pose.position.y = point_base.point.y
        pose.pose.position.z = point_base.point.z

        # Raw image angle (degrees) is stashed in orientation.z, not a true
        # quaternion component — the pick_place_server reads it directly.
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = angle_deg
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        self.get_logger().info(
            f'Cube pose in base frame: '
            f'x={pose.pose.position.x:.4f}, '
            f'y={pose.pose.position.y:.4f}, '
            f'z={pose.pose.position.z:.4f}')

        self.publish_marker(pose)

    # ────────────────────────────────────────────────────────────
    def publish_marker(self, pose):
        # pose.pose.orientation is NOT a real quaternion — publish_pose() stashes
        # the raw image angle (degrees) in .z for pick_place_server to read.
        # Feeding that straight to a Marker makes the RViz box snap/twitch every
        # frame as the raw angle jitters. Build an actual yaw quaternion instead
        # so the marker rotates smoothly and matches the cube's real orientation.
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'cube'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = pose.pose.position
        marker.pose.orientation = _yaw_quaternion(pose.pose.orientation.z)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
