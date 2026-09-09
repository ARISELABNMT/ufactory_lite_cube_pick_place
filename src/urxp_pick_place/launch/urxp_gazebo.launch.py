"""
URXP Gazebo simulation — single-launch demo: Gazebo (gz sim) + xArm Lite6 +
table + simulated RealSense + colored target cube + full pick-place pipeline.

  source /opt/ros/jazzy/setup.bash
  source ~/URXP_ws/install/setup.bash
  ros2 launch urxp_pick_place urxp_gazebo.launch.py

What this brings up (all in one Gazebo window):
  - Gazebo world with a table (xarm_gazebo/worlds/table_gz.world)
  - xArm Lite6 + vacuum gripper + wrist RealSense, spawned via xarm_moveit_config's
    existing gz_type=gz MoveIt+Gazebo launch chain (real ros2_control through
    gz_ros2_control, not a kinematic-only fake)
  - The 3-colored target cube (models/target_cube), spawned a few seconds later
  - cube_detector + pose_filter + gripper_sim_node + pick_place_server, reading
    the simulated camera and gripping the cube via a Gazebo DetachableJoint
    (see xarm_description's lite_vacuum_gripper.urdf.xacro)

The cube's spawn pose (cube_x/y/z/yaw args below) is a best-effort estimate
computed from the robot's known spawn pose in table_gz.world — nudge it via
launch arguments if the cube doesn't land on the table surface / in camera view.

To trigger a pick-place once the cube is detected and stable:
  ros2 topic pub --once /urxp/execute std_msgs/msg/Bool "{data: true}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cube_x = LaunchConfiguration('cube_x')
    cube_y = LaunchConfiguration('cube_y')
    cube_z = LaunchConfiguration('cube_z')
    cube_yaw = LaunchConfiguration('cube_yaw')

    detector_config = PathJoinSubstitution([
        FindPackageShare('urxp_pick_place'), 'config', 'detector_params.yaml'])
    pick_place_config = PathJoinSubstitution([
        FindPackageShare('urxp_pick_place'), 'config', 'pick_place_params.yaml'])
    cube_sdf = PathJoinSubstitution([
        FindPackageShare('urxp_pick_place'), 'models', 'target_cube', 'model.sdf'])

    # ── Gazebo + robot + MoveIt + camera bridge (existing, proven vendor chain) ──
    robot_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_gazebo.launch.py'
        ])),
        launch_arguments={
            'dof': '6',
            'robot_type': 'lite',
            'hw_ns': 'ufactory',
            'no_gui_ctrl': 'false',
            'gz_type': 'gz',
            'add_vacuum_gripper': 'true',
            'add_realsense_d435i': 'true',
            'add_d435i_links': 'true',
        }.items(),
    )

    # ── Spawn the target cube a few seconds after the world/robot come up ───
    spawn_cube_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', cube_sdf,
            '-name', 'target_cube',
            '-x', cube_x, '-y', cube_y, '-z', cube_z,
            '-Y', cube_yaw,
        ],
        parameters=[{'use_sim_time': True}],
    )
    delayed_cube_spawn = TimerAction(period=8.0, actions=[spawn_cube_node])

    # ── Bridge the vacuum-grip attach/detach triggers into Gazebo ───────────
    gripper_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/urxp/gripper_attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/urxp/gripper_detach@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
        output='screen',
    )

    # ── Cube color detector + pose filter (sim camera topic names) ──────────
    cube_detector_node = Node(
        package='urxp_pick_place',
        executable='cube_detector',
        name='cube_detector',
        parameters=[detector_config, {
            'use_sim_time': True,
            'color_image_topic': '/camera/color/image_raw',
            'color_info_topic': '/camera/color/camera_info',
            'depth_image_topic': '/camera/depth/image',
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    pose_filter_node = Node(
        package='urxp_pick_place',
        executable='pose_filter',
        name='pose_filter',
        parameters=[detector_config, {'use_sim_time': True}],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # ── Simulated vacuum gripper (Gazebo DetachableJoint, not Tool GPIO) ─────
    gripper_sim_node = Node(
        package='urxp_pick_place',
        executable='gripper_sim_node',
        name='urxp_gripper_sim_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Pick-place server ─────────────────────────────────────────────────
    pick_place_node = Node(
        package='urxp_pick_place',
        executable='pick_place_server',
        name='urxp_pick_place_server',
        parameters=[pick_place_config, {'use_sim_time': True}],
        output='screen',
    )

    # Give Gazebo + the robot + the simulated camera time to fully come up
    # before starting detection (matches the 6s margin used for the real
    # camera in urxp_pick_place.launch.py, plus the cube's own 8s spawn delay).
    delayed_pipeline = TimerAction(
        period=12.0,
        actions=[cube_detector_node, pose_filter_node, gripper_sim_node, pick_place_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument('cube_x', default_value='-0.2',
                               description='Target cube spawn X in world frame (best-effort estimate — nudge after visual check)'),
        DeclareLaunchArgument('cube_y', default_value='-0.22',
                               description='Target cube spawn Y in world frame'),
        DeclareLaunchArgument('cube_z', default_value='1.05',
                               description='Target cube spawn Z in world frame (table surface height + half cube)'),
        DeclareLaunchArgument('cube_yaw', default_value='0.0',
                               description='Target cube spawn yaw (radians) — rotate to test orientation-aware grasping'),
        robot_moveit_gazebo_launch,
        gripper_bridge_node,
        delayed_cube_spawn,
        delayed_pipeline,
    ])
