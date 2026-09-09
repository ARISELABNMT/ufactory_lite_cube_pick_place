"""
URXP Web UI server — a local dashboard for the pick-and-place system.

Run (from a sourced terminal, same as any other URXP_ws command):
  source /opt/ros/jazzy/setup.bash
  source ~/URXP_ws/install/setup.bash
  ros2 run urxp_web_ui web_ui_server

Then open http://localhost:8080 (or http://<this-pc's-LAN-IP>:8080 from
another device on the same network).

Env vars (all optional):
  URXP_WEB_UI_PORT   default 8080
  URXP_ROBOT_IP      default 192.168.1.165 (passed to urxp_robot.launch.py)
"""

import http.server
import json
import os
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory

from urxp_web_ui import config_store
from urxp_web_ui.launch_manager import ManagedLaunch
from urxp_web_ui.ros_bridge import RosBridge, spin_in_background

STATIC_DIR = Path(get_package_share_directory('urxp_web_ui')) / 'static'

STATIC_FILES = {
    '/': ('index.html', 'text/html'),
    '/index.html': ('index.html', 'text/html'),
    '/app.js': ('app.js', 'text/javascript'),
    '/style.css': ('style.css', 'text/css'),
}


def make_handler(bridge: RosBridge, robot_launch: ManagedLaunch, pipeline_launch: ManagedLaunch):
    class Handler(http.server.BaseHTTPRequestHandler):
        server_version = 'URXPWebUI/1.0'

        def log_message(self, fmt, *args):
            pass  # keep the terminal quiet; the UI's own log panel covers this

        # ── helpers ──────────────────────────────────────────
        def _send_json(self, obj, status=200):
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self):
            length = int(self.headers.get('Content-Length', 0))
            if length == 0:
                return {}
            try:
                return json.loads(self.rfile.read(length))
            except Exception:
                return {}

        def _send_static(self, filename, content_type):
            path = STATIC_DIR / filename
            try:
                data = path.read_bytes()
            except FileNotFoundError:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _stream_mjpeg(self, which):
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    frame = bridge.latest_jpeg(which)
                    if frame is not None:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.08)  # ~12 fps cap regardless of source rate
            except (BrokenPipeError, ConnectionResetError):
                pass

        # ── GET ──────────────────────────────────────────────
        def do_GET(self):
            if self.path in STATIC_FILES:
                filename, ctype = STATIC_FILES[self.path]
                self._send_static(filename, ctype)
                return

            if self.path == '/api/status':
                snap = bridge.snapshot()
                snap['robot_running'] = robot_launch.is_running()
                snap['pipeline_running'] = pipeline_launch.is_running()
                self._send_json(snap)
                return

            if self.path == '/api/place_positions':
                live = bridge.get_place_positions()
                if live:
                    self._send_json({'positions': live, 'source': 'live (running node)'})
                else:
                    self._send_json({'positions': config_store.read_place_positions(),
                                      'source': 'config file (pipeline not running)'})
                return

            if self.path == '/stream/debug':
                self._stream_mjpeg('debug')
                return

            if self.path == '/stream/raw':
                self._stream_mjpeg('raw')
                return

            self.send_error(404)

        # ── POST ─────────────────────────────────────────────
        def do_POST(self):
            if self.path == '/api/execute':
                bridge.trigger_execute()
                self._send_json({'ok': True, 'message': 'Execute triggered'})
                return

            if self.path == '/api/launch/robot/start':
                ok, msg = robot_launch.start()
                self._send_json({'ok': ok, 'message': msg})
                return
            if self.path == '/api/launch/robot/stop':
                ok, msg = robot_launch.stop()
                self._send_json({'ok': ok, 'message': msg})
                return
            if self.path == '/api/launch/pipeline/start':
                ok, msg = pipeline_launch.start()
                self._send_json({'ok': ok, 'message': msg})
                return
            if self.path == '/api/launch/pipeline/stop':
                ok, msg = pipeline_launch.stop()
                self._send_json({'ok': ok, 'message': msg})
                return

            if self.path == '/api/place_positions':
                body = self._read_json()
                positions = {c: body[c] for c in ('red', 'green', 'blue')
                             if c in body and isinstance(body[c], list) and len(body[c]) == 3}
                if not positions:
                    self._send_json({'ok': False, 'message': 'No valid color positions in request'}, 400)
                    return
                live_ok, live_msg = bridge.set_place_positions(positions)
                file_ok, file_msg = config_store.write_place_positions(positions)
                self._send_json({
                    'ok': file_ok,  # file write is the one that must succeed
                    'message': f'{file_msg} — live node: {live_msg}',
                })
                return

            self.send_error(404)

    return Handler


def main():
    rclpy.init()
    bridge = RosBridge()
    spin_in_background(bridge)

    robot_ip = os.environ.get('URXP_ROBOT_IP', '192.168.1.165')
    robot_launch = ManagedLaunch(
        'Robot + MoveIt',
        ['ros2', 'launch', 'urxp_pick_place', 'urxp_robot.launch.py', f'robot_ip:={robot_ip}'],
        'urxp_pick_place urxp_robot.launch.py')
    pipeline_launch = ManagedLaunch(
        'Camera + Pipeline',
        ['ros2', 'launch', 'urxp_pick_place', 'urxp_pick_place.launch.py'],
        'urxp_pick_place urxp_pick_place.launch.py')

    port = int(os.environ.get('URXP_WEB_UI_PORT', '8080'))
    handler_cls = make_handler(bridge, robot_launch, pipeline_launch)
    httpd = http.server.ThreadingHTTPServer(('0.0.0.0', port), handler_cls)

    print(f'URXP Web UI: http://localhost:{port}  (Ctrl+C to stop)')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
