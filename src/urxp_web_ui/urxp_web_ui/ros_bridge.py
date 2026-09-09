"""
rclpy node backing the web UI: mirrors the pick-place topics into
thread-safe in-memory state the HTTP handler can read, and offers helpers
to trigger execution and to get/set the live pick_place_server's
place_position_<color> parameters.
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

PICK_PLACE_NODE = 'urxp_pick_place_server'
COLORS = ('red', 'green', 'blue')


def _wait_future(future, timeout_sec: float) -> bool:
    """Block the calling thread until future is done — safe to call from the
    HTTP handler thread while the node's own executor spins elsewhere."""
    event = threading.Event()
    future.add_done_callback(lambda _f: event.set())
    return event.wait(timeout=timeout_sec)


def _jpeg_bytes(cv_image) -> bytes | None:
    ok, buf = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return buf.tobytes() if ok else None


class RosBridge(Node):
    def __init__(self):
        super().__init__('urxp_web_ui')
        self._cbg = ReentrantCallbackGroup()
        self._bridge = CvBridge()
        self._lock = threading.Lock()

        # ── Mirrored state ──────────────────────────────
        self.status_text = '(no status yet — start the pipeline)'
        self.status_time = 0.0
        self.busy = False
        self.last_result: bool | None = None
        self.detected_color = 'unknown'
        self.detected_pose = None  # (x, y, z) or None
        self._debug_jpeg: bytes | None = None
        self._raw_jpeg: bytes | None = None

        self.create_subscription(String, '/urxp/status', self._cb_status, 10,
                                  callback_group=self._cbg)
        self.create_subscription(Bool, '/urxp/result', self._cb_result, 10,
                                  callback_group=self._cbg)
        self.create_subscription(String, '/urxp/cube_color', self._cb_color, 10,
                                  callback_group=self._cbg)
        self.create_subscription(PoseStamped, '/urxp/cube_pose_filtered', self._cb_pose, 10,
                                  callback_group=self._cbg)
        self.create_subscription(Image, '/urxp/debug_image', self._cb_debug_image, 1,
                                  callback_group=self._cbg)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self._cb_raw_image, 1,
                                  callback_group=self._cbg)

        self._execute_pub = self.create_publisher(Bool, '/urxp/execute', 10)

        self._get_params_client = self.create_client(
            GetParameters, f'/{PICK_PLACE_NODE}/get_parameters', callback_group=self._cbg)
        self._set_params_client = self.create_client(
            SetParameters, f'/{PICK_PLACE_NODE}/set_parameters', callback_group=self._cbg)

    # ── Subscriptions ──────────────────────────────────────
    def _cb_status(self, msg: String):
        with self._lock:
            self.status_text = msg.data
            self.status_time = time.time()
            text = msg.data
            if text.startswith('Step '):
                self.busy = True
            elif text.startswith('SUCCESS') or text.startswith('FAILED') or 'IDLE' in text:
                self.busy = False

    def _cb_result(self, msg: Bool):
        with self._lock:
            self.last_result = msg.data
            self.busy = False

    def _cb_color(self, msg: String):
        with self._lock:
            self.detected_color = msg.data

    def _cb_pose(self, msg: PoseStamped):
        p = msg.pose.position
        with self._lock:
            self.detected_pose = (p.x, p.y, p.z)

    def _cb_debug_image(self, msg: Image):
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        jpeg = _jpeg_bytes(cv_img)
        if jpeg is not None:
            with self._lock:
                self._debug_jpeg = jpeg

    def _cb_raw_image(self, msg: Image):
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        jpeg = _jpeg_bytes(cv_img)
        if jpeg is not None:
            with self._lock:
                self._raw_jpeg = jpeg

    # ── Reads for the HTTP layer ────────────────────────────
    def snapshot(self) -> dict:
        with self._lock:
            return {
                'status_text': self.status_text,
                'status_age_sec': (time.time() - self.status_time) if self.status_time else None,
                'busy': self.busy,
                'last_result': self.last_result,
                'detected_color': self.detected_color,
                'detected_pose': self.detected_pose,
            }

    def latest_jpeg(self, which: str) -> bytes | None:
        with self._lock:
            return self._debug_jpeg if which == 'debug' else self._raw_jpeg

    # ── Actions ──────────────────────────────────────────
    def trigger_execute(self):
        msg = Bool()
        msg.data = True
        self._execute_pub.publish(msg)

    # ── pick_place_server parameter bridge ──────────────────
    def get_place_positions(self, timeout_sec: float = 2.0) -> dict | None:
        """Returns {'red': [x,y,z], ...} from the live node, or None if the
        node (or its parameter service) isn't up."""
        if not self._get_params_client.wait_for_service(timeout_sec=1.0):
            return None
        names = [f'place_position_{c}' for c in COLORS]
        req = GetParameters.Request()
        req.names = names
        future = self._get_params_client.call_async(req)
        if not _wait_future(future, timeout_sec) or future.result() is None:
            return None
        result = {}
        for color, val in zip(COLORS, future.result().values):
            if val.type == ParameterType.PARAMETER_DOUBLE_ARRAY and len(val.double_array_value) == 3:
                result[color] = list(val.double_array_value)
        return result or None

    def set_place_positions(self, positions: dict) -> tuple[bool, str]:
        """positions: {'red': [x,y,z], ...} (subset ok). Returns (ok, message)."""
        if not self._set_params_client.wait_for_service(timeout_sec=1.0):
            return False, 'pick_place_server is not running — value saved to file only'
        params = []
        for color, xyz in positions.items():
            if color not in COLORS:
                continue
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                 double_array_value=[float(v) for v in xyz])
            params.append(Parameter(name=f'place_position_{color}', value=pv))
        if not params:
            return False, 'No valid colors given'
        req = SetParameters.Request()
        req.parameters = params
        future = self._set_params_client.call_async(req)
        if not _wait_future(future, 3.0) or future.result() is None:
            return False, 'Timed out setting live parameters'
        for r in future.result().results:
            if not r.successful:
                return False, f'Rejected: {r.reason}'
        return True, 'Applied to the running node'


def spin_in_background(node: Node):
    """Runs rclpy.spin(node) in a daemon thread; returns the thread."""
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return thread
