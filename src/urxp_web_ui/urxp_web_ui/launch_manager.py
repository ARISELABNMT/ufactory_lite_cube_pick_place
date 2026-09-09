"""
Starts/stops `ros2 launch` processes as detached process groups, so a Stop
button can kill the whole launch tree, the same way Ctrl+C in a terminal
does. Also detects and can stop a matching launch that was started from an
ordinary terminal before the web UI existed — matched by command line via
pgrep, not just PIDs this process itself spawned — since that's exactly
the workflow this UI is meant to take over.
"""

import os
import signal
import subprocess
import threading
import time


def _pids_matching(pattern: str) -> list[int]:
    try:
        out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True)
        return [int(p) for p in out.stdout.split()]
    except Exception:
        return []


def _kill_pid_group(pid: int, timeout_sec: float = 8.0):
    try:
        pgid = os.getpgid(pid)
    except ProcessLookupError:
        return
    for sig, wait in ((signal.SIGINT, timeout_sec), (signal.SIGTERM, 1.5), (signal.SIGKILL, 0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        deadline = time.time() + wait
        while time.time() < deadline:
            if not _pgid_alive(pgid):
                return
            time.sleep(0.2)


def _pgid_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


class ManagedLaunch:
    """One named, start/stoppable `ros2 launch` invocation. `match_pattern`
    is a substring of the full command line (e.g. "urxp_pick_place
    urxp_robot.launch.py") used to detect/stop a matching process regardless
    of whether this server or a plain terminal started it."""

    def __init__(self, name: str, argv: list, match_pattern: str):
        self.name = name
        self.argv = argv
        self.match_pattern = match_pattern
        self._proc: subprocess.Popen | None = None
        self._lock = threading.Lock()

    def is_running(self) -> bool:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return True
        return bool(_pids_matching(self.match_pattern))

    def started_by_this_server(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def start(self) -> tuple[bool, str]:
        if self.is_running():
            return False, f'{self.name} is already running'
        with self._lock:
            try:
                # New process group (os.setsid) so stop() can signal every
                # child node ros2 launch spawns, not just the launch process.
                self._proc = subprocess.Popen(
                    self.argv,
                    preexec_fn=os.setsid,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception as e:
                return False, f'Failed to start {self.name}: {e}'
        return True, f'{self.name} starting (pid {self._proc.pid})'

    def stop(self) -> tuple[bool, str]:
        pids = _pids_matching(self.match_pattern)
        with self._lock:
            if self._proc is not None and self._proc.poll() is None and self._proc.pid not in pids:
                pids.append(self._proc.pid)
        if not pids:
            return False, f'{self.name} is not running'
        for pid in pids:
            _kill_pid_group(pid)
        return True, f'{self.name} stopped'
