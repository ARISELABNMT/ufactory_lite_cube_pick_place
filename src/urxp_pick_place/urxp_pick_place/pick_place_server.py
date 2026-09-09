"""
Standalone Pick-and-Place Server for xArm Lite6 + RealSense.

Picks a colored cube (auto-detected by cube_detector, or a manually
published pose), grasps it with an orientation matched to the cube's
detected yaw, and places it at one of three predefined positions selected
by the cube's detected color. Uses MoveIt's MoveGroup action client and a
separate gripper node (real Tool-GPIO or Gazebo sim) for vacuum control.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Commands in:
  /urxp/pick_pose   geometry_msgs/PoseStamped   — manual pick override (optional)
  /urxp/execute     std_msgs/Bool (True)        — trigger execution

Status out:
  /urxp/status      std_msgs/String             — state machine status
  /urxp/result      std_msgs/Bool               — True=success, False=fail
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Coordinate frame: link_base (robot base frame).
If no manual pick pose is published, the most recent /urxp/cube_pose_filtered
(camera detection) is used automatically. The place position is chosen from
config/pick_place_params.yaml (place_position_<color>) using the color last
reported on /urxp/cube_color — no place pose needs to be published.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    JointConstraint, PositionConstraint, OrientationConstraint,
    BoundingVolume, MoveItErrorCodes,
    CollisionObject, AttachedCollisionObject, PlanningScene,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory

CUBE_ID = 'target_cube'


def _wait_future(future, timeout_sec: float) -> bool:
    """Block the calling thread until future is done. Thread-safe (no executor needed)."""
    event = threading.Event()
    future.add_done_callback(lambda _f: event.set())
    return event.wait(timeout=timeout_sec)


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('urxp_pick_place_server')

        # ── Parameters (see config/pick_place_params.yaml) ─────
        self.declare_parameter('joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('observe_joints',
            [0.0, -0.5, 1.2, 0.0, 1.5, 0.0])
        self.declare_parameter('planning_group', 'lite6')
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('eef_link', 'link_eef')
        self.declare_parameter('pre_grasp_height', 0.15)
        self.declare_parameter('grasp_height_offset', 0.062)
        self.declare_parameter('place_height_offset', 0.068)
        self.declare_parameter('cartesian_step_m', 0.010)
        self.declare_parameter('cartesian_jump_threshold', 0.0)
        self.declare_parameter('approach_velocity', 0.2)
        self.declare_parameter('descent_velocity', 0.1)
        self.declare_parameter('lift_velocity', 0.1)
        self.declare_parameter('gripper_wait_sec', 2.0)
        self.declare_parameter('plan_timeout_sec', 10.0)
        self.declare_parameter('preview_wait_sec', 2.5)
        self.declare_parameter('cube_size_m', 0.05)
        self.declare_parameter('scene_update_interval_sec', 0.3)
        # Predefined place positions [x, y, z] in base_frame, selected by detected cube color.
        self.declare_parameter('place_position_red', [0.30, 0.25, 0.02])
        self.declare_parameter('place_position_green', [0.30, 0.0, 0.02])
        self.declare_parameter('place_position_blue', [0.30, -0.25, 0.02])

        self.JOINT_NAMES     = list(self.get_parameter('joint_names').value)
        self.OBSERVE_JOINTS  = list(self.get_parameter('observe_joints').value)
        self.PLANNING_GROUP  = self.get_parameter('planning_group').value
        self.BASE_FRAME      = self.get_parameter('base_frame').value
        self.EEF_LINK        = self.get_parameter('eef_link').value
        self.PRE_GRASP_HEIGHT    = self.get_parameter('pre_grasp_height').value
        self.GRASP_HEIGHT_OFFSET = self.get_parameter('grasp_height_offset').value
        self.PLACE_HEIGHT_OFFSET = self.get_parameter('place_height_offset').value
        self.CARTESIAN_STEP  = self.get_parameter('cartesian_step_m').value
        self.CARTESIAN_JUMP  = self.get_parameter('cartesian_jump_threshold').value
        self.APPROACH_VEL    = self.get_parameter('approach_velocity').value
        self.DESCENT_VEL     = self.get_parameter('descent_velocity').value
        self.LIFT_VEL        = self.get_parameter('lift_velocity').value
        self.GRIPPER_WAIT    = self.get_parameter('gripper_wait_sec').value
        self.PLAN_TIMEOUT_SEC = self.get_parameter('plan_timeout_sec').value
        self.PREVIEW_WAIT    = self.get_parameter('preview_wait_sec').value
        self.CUBE_SIZE       = self.get_parameter('cube_size_m').value
        self.SCENE_UPDATE_INTERVAL = self.get_parameter('scene_update_interval_sec').value
        self.PLACE_POSITIONS = {
            'red':   list(self.get_parameter('place_position_red').value),
            'green': list(self.get_parameter('place_position_green').value),
            'blue':  list(self.get_parameter('place_position_blue').value),
        }

        # Callback groups: subscriptions use reentrant so they don't block each other;
        # service/action clients use a separate group to avoid deadlock.
        self._sub_cbg = ReentrantCallbackGroup()
        self._cli_cbg = MutuallyExclusiveCallbackGroup()

        # ── State ──────────────────────────────
        self._pick_pose: PoseStamped | None = None      # manual override
        self._detected_pose: PoseStamped | None = None  # from cube_detector (camera)
        self._detected_color: str = 'unknown'
        self._joint_state: JointState | None = None
        self._busy = False
        self._last_scene_update_time: float = 0.0  # throttle camera scene updates

        # ── Subscribers ────────────────────────
        self.create_subscription(
            PoseStamped, '/urxp/pick_pose', self._cb_pick_pose, 10,
            callback_group=self._sub_cbg)
        self.create_subscription(
            Bool, '/urxp/execute', self._cb_execute, 10,
            callback_group=self._sub_cbg)
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint_state, 10,
            callback_group=self._sub_cbg)
        self.create_subscription(
            PoseStamped, '/urxp/cube_pose_filtered', self._cb_detected_pose, 10,
            callback_group=self._sub_cbg)
        self.create_subscription(
            String, '/urxp/cube_color', self._cb_detected_color, 10,
            callback_group=self._sub_cbg)

        # ── Publishers ─────────────────────────
        self._status_pub  = self.create_publisher(String, '/urxp/status', 10)
        self._result_pub  = self.create_publisher(Bool, '/urxp/result', 10)
        self._gripper_pub = self.create_publisher(Bool, '/urxp/gripper/command', 10)

        # ── Action / Service clients ───────────
        self._move_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self._cli_cbg)
        self._exec_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self._cli_cbg)
        self._cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=self._cli_cbg)
        self._scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene',
            callback_group=self._cli_cbg)

        self._publish_status('IDLE — waiting for pick/place poses')
        self.get_logger().info('URXP Pick-Place Server ready')

    # ──────────────────────────────────────────
    # Topic callbacks
    # ──────────────────────────────────────────
    def _cb_pick_pose(self, msg: PoseStamped):
        self._pick_pose = msg
        p = msg.pose.position
        self.get_logger().info(f'Pick pose received: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        self._scene_add_cube_async(p.x, p.y, p.z)
        self._publish_status(
            f'Pick pose set: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})\n'
            f'Cube added to MoveIt scene.\n'
            f'Send /urxp/execute=True when ready.')

    def _cb_joint_state(self, msg: JointState):
        self._joint_state = msg

    def _cb_detected_pose(self, msg: PoseStamped):
        """Auto-update pick pose from camera detection (used when no manual pose set)."""
        self._detected_pose = msg
        if self._busy:
            # Never update the planning scene while executing — the camera running at 30 Hz
            # would continuously re-add the cube as a world collision object while it is
            # attached to the EEF, causing START_STATE_IN_COLLISION during planning.
            return
        # Rate-limit scene updates (default ~3 Hz, see scene_update_interval_sec):
        # at the camera's full ~30 fps the ApplyPlanningScene service call queue
        # fills up, causing _scene_remove_cube_blocking to time out on the next
        # task. Too slow here instead reads as a laggy cube in RViz when moving
        # it by hand — tune scene_update_interval_sec to trade one off the other.
        now = time.time()
        if now - self._last_scene_update_time < self.SCENE_UPDATE_INTERVAL:
            return
        self._last_scene_update_time = now
        p = msg.pose.position
        self._scene_add_cube_async(p.x, p.y, p.z)
        self._publish_status(
            f'[CAMERA] Cube detected: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})\n'
            f'Color: {self._detected_color}\n'
            f'Set place pose then send /urxp/execute=True')

    def _cb_detected_color(self, msg: String):
        self._detected_color = msg.data
        self.get_logger().info(f'Cube color: {msg.data}')

    def _cb_execute(self, msg: Bool):
        if not msg.data:
            return
        if self._busy:
            self.get_logger().warn('Already executing — ignoring trigger')
            return
        active_pick = self._pick_pose or self._detected_pose
        if active_pick is None:
            self._publish_status('ERROR: No pick pose — publish to /urxp/pick_pose or wait for camera detection')
            return
        place_xyz = self.PLACE_POSITIONS.get(self._detected_color)
        if place_xyz is None:
            self._publish_status(
                f'ERROR: Cube color "{self._detected_color}" has no configured place '
                f'position (known: {", ".join(self.PLACE_POSITIONS)})')
            return
        self._active_pick = active_pick
        self._active_place_xyz = place_xyz
        self._active_color = self._detected_color
        self._busy = True
        threading.Thread(target=self._run_pick_place, daemon=True).start()

    # ──────────────────────────────────────────
    # Main pick-and-place sequence (background thread)
    # ──────────────────────────────────────────
    def _run_pick_place(self):
        try:
            pick_pos = self._active_pick.pose.position
            place_x, place_y, place_z = self._active_place_xyz
            grasp_orientation = self._grasp_orientation(self._active_pick.pose.orientation.z)

            self.get_logger().info(
                f'Starting pick-place: pick=({pick_pos.x:.3f},{pick_pos.y:.3f},{pick_pos.z:.3f})'
                f' color={self._active_color} place=({place_x:.3f},{place_y:.3f},{place_z:.3f})'
                f' source={"camera" if self._pick_pose is None else "manual"}')

            self._publish_status('Step 1/7: Moving to observe position...')
            if not self._move_to_joints(self.OBSERVE_JOINTS, self.APPROACH_VEL):
                raise RuntimeError('Failed to reach observe position')

            self._publish_status('Step 2/7: Moving above pick location...')
            if not self._move_to_pose(
                    self._make_pose(pick_pos.x, pick_pos.y, pick_pos.z + self.PRE_GRASP_HEIGHT,
                                     grasp_orientation),
                    self.APPROACH_VEL):
                raise RuntimeError('Failed to reach pick approach position')

            # Remove cube from scene so the descent can approach it without collision.
            self._scene_remove_cube_blocking()

            self._publish_status('Step 3/7: Descending to pick...')
            if not self._cartesian_move(
                    [self._make_pose(pick_pos.x, pick_pos.y,
                                     pick_pos.z + self.GRASP_HEIGHT_OFFSET, grasp_orientation)],
                    self.DESCENT_VEL, avoid_collisions=False):
                raise RuntimeError('Cartesian descent to pick failed')

            self._publish_status(f'Step 4/7: Closing gripper (grasping {self._active_color} cube)...')
            self._gripper_cmd(open=False)
            self._scene_attach_cube_blocking()

            self._publish_status('Step 5/7: Lifting from pick...')
            if not self._cartesian_move(
                    [self._make_pose(pick_pos.x, pick_pos.y,
                                     pick_pos.z + self.PRE_GRASP_HEIGHT, grasp_orientation)],
                    self.LIFT_VEL, avoid_collisions=False):
                raise RuntimeError('Failed to lift from pick')

            self._publish_status(f'Step 6/7: Moving to {self._active_color} place location and descending...')
            if not self._move_to_pose(
                    self._make_pose(place_x, place_y, place_z + self.PRE_GRASP_HEIGHT),
                    self.APPROACH_VEL):
                raise RuntimeError('Failed to reach place approach position')

            if not self._cartesian_move(
                    [self._make_pose(place_x, place_y, place_z + self.PLACE_HEIGHT_OFFSET)],
                    self.DESCENT_VEL, avoid_collisions=False):
                raise RuntimeError('Cartesian descent to place failed')

            self._publish_status('Step 7/7: Releasing and lifting...')
            self._gripper_cmd(open=True)
            # Detach only — do NOT re-add cube to world yet.
            # Re-adding it here puts a collision object directly under the EEF,
            # causing START_STATE_IN_COLLISION and blocking the lift and return moves.
            self._scene_detach_only_blocking()
            if not self._cartesian_move(
                    [self._make_pose(place_x, place_y, place_z + self.PRE_GRASP_HEIGHT)],
                    self.LIFT_VEL, avoid_collisions=False):
                raise RuntimeError('Cartesian lift from place failed')

            if not self._move_to_joints(self.OBSERVE_JOINTS, self.APPROACH_VEL):
                raise RuntimeError('Failed to return to observe position')

            # Cube is safely placed — add it to the scene for visualization.
            self._scene_add_cube_async(place_x, place_y, place_z)
            self._publish_status('SUCCESS: Pick-place complete. Ready for next command.')
            self._publish_result(success=True)
            self.get_logger().info('Pick-place sequence SUCCEEDED')

        except Exception as e:
            self.get_logger().error(f'Pick-place FAILED: {e}')
            self._publish_status(f'FAILED: {e}')
            self._publish_result(success=False)
            try:
                self._gripper_cmd(open=True)
                self._move_to_joints(self.OBSERVE_JOINTS, self.APPROACH_VEL * 0.5)
            except Exception:
                pass
        finally:
            self._busy = False

    # ──────────────────────────────────────────
    # Motion helpers (all use threading.Event — safe from background threads)
    # ──────────────────────────────────────────
    def _move_to_joints(self, joint_values: list, velocity_scale: float = 0.3) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        constraints = Constraints()
        for name, value in zip(self.JOINT_NAMES, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request = self._base_request(constraints, velocity_scale)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True   # MoveGroup publishes orange robot to RViz

        send_future = self._move_client.send_goal_async(goal)
        if not _wait_future(send_future, self.PLAN_TIMEOUT_SEC + 5):
            self.get_logger().error('Joint move: send_goal timed out')
            return False
        if not send_future.result().accepted:
            self.get_logger().error('Joint move: goal rejected')
            return False

        result_future = send_future.result().get_result_async()
        if not _wait_future(result_future, 60.0):
            self.get_logger().error('Joint move: result timed out')
            return False

        code = result_future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'Joint move plan failed, error code: {code}')
            return False

        planned = result_future.result().result.planned_trajectory
        time.sleep(self.PREVIEW_WAIT)
        return self._execute_trajectory(planned)

    def _move_to_pose(self, pose: Pose, velocity_scale: float = 0.3) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        constraints = self._pose_constraints(pose)
        request = self._base_request(constraints, velocity_scale)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True   # MoveGroup publishes orange robot to RViz

        send_future = self._move_client.send_goal_async(goal)
        if not _wait_future(send_future, self.PLAN_TIMEOUT_SEC + 5):
            self.get_logger().error('Pose move: send_goal timed out')
            return False
        if not send_future.result().accepted:
            self.get_logger().error('Pose move: goal rejected')
            return False

        result_future = send_future.result().get_result_async()
        if not _wait_future(result_future, 60.0):
            self.get_logger().error('Pose move: result timed out')
            return False

        code = result_future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'Pose move plan failed, error code: {code}')
            return False

        planned = result_future.result().result.planned_trajectory
        time.sleep(self.PREVIEW_WAIT)
        return self._execute_trajectory(planned)

    def _pose_move_execute(self, pose: Pose, velocity_scale: float = 0.1) -> bool:
        """Plan + execute a pose goal via MoveGroup with no preview delay."""
        if not self._move_client.wait_for_server(timeout_sec=15.0):
            return False

        constraints = self._pose_constraints(pose)
        request = self._base_request(constraints, velocity_scale)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        send_future = self._move_client.send_goal_async(goal)
        if not _wait_future(send_future, self.PLAN_TIMEOUT_SEC + 5) or not send_future.result().accepted:
            return False
        result_future = send_future.result().get_result_async()
        if not _wait_future(result_future, 60.0):
            return False
        return result_future.result().result.error_code.val == MoveItErrorCodes.SUCCESS

    def _pose_constraints(self, pose: Pose) -> Constraints:
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]
        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [pose]

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.BASE_FRAME
        pos_constraint.link_name = self.EEF_LINK
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.BASE_FRAME
        ori_constraint.link_name = self.EEF_LINK
        ori_constraint.orientation = pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pos_constraint]
        constraints.orientation_constraints = [ori_constraint]
        return constraints

    def _base_request(self, constraints: Constraints, velocity_scale: float) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = self.PLANNING_GROUP
        request.goal_constraints = [constraints]
        request.allowed_planning_time = self.PLAN_TIMEOUT_SEC
        request.max_velocity_scaling_factor = velocity_scale
        request.max_acceleration_scaling_factor = velocity_scale * 0.5
        request.num_planning_attempts = 5
        if self._joint_state:
            request.start_state.joint_state = self._joint_state
        return request

    def _execute_trajectory(self, trajectory: RobotTrajectory) -> bool:
        if not self._exec_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('ExecuteTrajectory action server not available')
            return False

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = trajectory

        exec_future = self._exec_client.send_goal_async(exec_goal)
        if not _wait_future(exec_future, 10.0):
            self.get_logger().error('ExecuteTrajectory: send_goal timed out')
            return False
        if not exec_future.result().accepted:
            self.get_logger().error('ExecuteTrajectory: goal rejected')
            return False

        result_future = exec_future.result().get_result_async()
        if not _wait_future(result_future, 60.0):
            self.get_logger().error('ExecuteTrajectory: result timed out')
            return False

        code = result_future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'ExecuteTrajectory failed, code: {code}')
            return False
        return True

    def _cartesian_move(self, waypoints: list, velocity_scale: float = 0.1, avoid_collisions: bool = True) -> bool:
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('GetCartesianPath service not available')
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = self.BASE_FRAME
        req.group_name = self.PLANNING_GROUP
        req.link_name = self.EEF_LINK
        req.waypoints = waypoints
        req.max_step = self.CARTESIAN_STEP
        req.jump_threshold = self.CARTESIAN_JUMP
        req.avoid_collisions = avoid_collisions
        if self._joint_state:
            req.start_state.joint_state = self._joint_state

        cart_future = self._cartesian_client.call_async(req)
        if not _wait_future(cart_future, 10.0):
            self.get_logger().error('GetCartesianPath timed out')
            return False

        result = cart_future.result()
        if result.fraction < 0.9:
            self.get_logger().warn(
                f'Cartesian path only {result.fraction*100:.0f}% — falling back to joint-space move')
            # Robot IK can't maintain a straight line for the full path at this position.
            # Fall back to a single joint-space move to the final waypoint (no preview).
            if waypoints:
                return self._pose_move_execute(waypoints[-1], velocity_scale)
            return False

        traj: JointTrajectory = result.solution.joint_trajectory
        if velocity_scale < 1.0:
            factor = 1.0 / velocity_scale
            for pt in traj.points:
                # Convert total time to ns, scale, then split back to sec+nanosec.
                # Direct nanosec multiplication overflows uint32 at high scale factors.
                total_ns = int((pt.time_from_start.sec * 1_000_000_000
                                + pt.time_from_start.nanosec) * factor)
                pt.time_from_start.sec = total_ns // 1_000_000_000
                pt.time_from_start.nanosec = total_ns % 1_000_000_000
                pt.velocities = [v / factor for v in pt.velocities]
                pt.accelerations = [a / (factor * factor) for a in pt.accelerations]

        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        return self._execute_trajectory(robot_traj)

    # ──────────────────────────────────────────
    # MoveIt Planning Scene helpers
    # ──────────────────────────────────────────
    def _scene_add_cube_async(self, x: float, y: float, z: float):
        """Fire-and-forget — safe from subscription callbacks (never blocks)."""
        if not self._scene_client.wait_for_service(timeout_sec=1.0):
            return
        req = ApplyPlanningScene.Request()
        req.scene = self._build_cube_scene(x, y, z)
        future = self._scene_client.call_async(req)
        future.add_done_callback(
            lambda _f: self.get_logger().info(
                f'Cube added to scene at ({x:.3f},{y:.3f},{z:.3f})'))

    def _scene_remove_cube_blocking(self):
        """Remove cube from world so the gripper can descend without collision."""
        obj = CollisionObject()
        obj.id = CUBE_ID
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self._apply_scene_blocking(scene)

    def _scene_attach_cube_blocking(self):
        """Blocking — call from background thread only."""
        attached = AttachedCollisionObject()
        attached.link_name = self.EEF_LINK
        attached.object.id = CUBE_ID
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = [self.EEF_LINK, 'link6']
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        self._apply_scene_blocking(scene)

    def _scene_detach_only_blocking(self):
        """Detach cube from EEF and remove it from world — do NOT re-add to world.
        Re-adding here causes START_STATE_IN_COLLISION for the post-place lift."""
        detach = AttachedCollisionObject()
        detach.link_name = self.EEF_LINK
        detach.object.id = CUBE_ID
        detach.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects = [detach]
        self._apply_scene_blocking(scene)

        obj = CollisionObject()
        obj.id = CUBE_ID
        obj.operation = CollisionObject.REMOVE
        scene2 = PlanningScene()
        scene2.is_diff = True
        scene2.world.collision_objects = [obj]
        self._apply_scene_blocking(scene2)

    def _build_cube_scene(self, x: float, y: float, z: float) -> PlanningScene:
        obj = CollisionObject()
        obj.header.frame_id = self.BASE_FRAME
        obj.id = CUBE_ID
        obj.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.CUBE_SIZE, self.CUBE_SIZE, self.CUBE_SIZE]
        obj.primitives = [box]
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z + self.CUBE_SIZE / 2.0)
        pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        obj.primitive_poses = [pose]
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        return scene

    def _apply_scene_blocking(self, scene: PlanningScene):
        if not self._scene_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('ApplyPlanningScene not available — skipping')
            return
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._scene_client.call_async(req)
        if not _wait_future(future, 10.0):
            self.get_logger().error(
                'ApplyPlanningScene timed out — service queue may be backed up. '
                'Check camera rate-limiting.')

    # ──────────────────────────────────────────
    # Utility helpers
    # ──────────────────────────────────────────
    def _gripper_cmd(self, open: bool):
        msg = Bool()
        msg.data = open
        self._gripper_pub.publish(msg)
        time.sleep(self.GRIPPER_WAIT)

    def _make_pose(self, x: float, y: float, z: float, orientation: Quaternion | None = None) -> Pose:
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        # Default: 180 degrees around X -> EEF pointing straight down.
        pose.orientation = orientation if orientation is not None else Quaternion(w=0.0, x=1.0, y=0.0, z=0.0)
        return pose

    def _grasp_orientation(self, raw_angle_deg: float) -> Quaternion:
        """Downward-pointing orientation additionally yawed to match the cube's
        detected rotation, so the gripper lines up with the cube's edges.
        raw_angle_deg comes from cube_detector's cv2.minAreaRect (image-plane
        angle, stashed in pose.orientation.z) — folded to [-45, 45] deg by cube
        symmetry, then composed with the 180-degree "point down" rotation:
        q = yaw(about Z) * down(180 deg about X) = (x=cos(yaw/2), y=sin(yaw/2), z=0, w=0).
        """
        folded = raw_angle_deg % 90.0
        if folded > 45.0:
            folded -= 90.0
        yaw = math.radians(folded)
        return Quaternion(x=math.cos(yaw / 2.0), y=math.sin(yaw / 2.0), z=0.0, w=0.0)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f'[STATUS] {text}')

    def _publish_result(self, success: bool):
        msg = Bool()
        msg.data = success
        self._result_pub.publish(msg)


def main():
    rclpy.init()
    node = PickPlaceServer()
    # MultiThreadedExecutor lets background threads call services/actions
    # while subscription callbacks are processed concurrently.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
