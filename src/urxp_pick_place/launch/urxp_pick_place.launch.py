"""
URXP pick-place pipeline — RealSense camera + cube detector/filter + gripper + pick-place server.

Terminal 2 of the URXP pick-place workflow (after urxp_robot.launch.py is up):
  source /opt/ros/jazzy/setup.bash
  source ~/URXP_ws/install/setup.bash
  ros2 launch urxp_pick_place urxp_pick_place.launch.py

Camera TF (link_eef -> camera_link -> ...) is published by the robot URDF
(add_realsense_d435i:=true in urxp_robot.launch.py), so no separate static
transform is needed here.

Pick pose and color are auto-detected from the RealSense camera. Place
position is chosen automatically from config/pick_place_params.yaml
(place_position_<color>). To trigger a task:
  ros2 topic pub --once /urxp/execute std_msgs/msg/Bool "{data: true}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')

    detector_config = PathJoinSubstitution([
        FindPackageShare('urxp_pick_place'), 'config', 'detector_params.yaml'])
    pick_place_config = PathJoinSubstitution([
        FindPackageShare('urxp_pick_place'), 'config', 'pick_place_params.yaml'])

    # ── RealSense camera (color + aligned depth) ────────────────────────────
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'enable_gyro': False,
            'enable_accel': False,
            'unite_imu_method': 0,
            'enable_infra1': False,
            'enable_infra2': False,
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'publish_tf': False,
        }],
        output='screen',
    )

    # ── Cube color detector + pose filter ───────────────────────────────────
    cube_detector_node = Node(
        package='urxp_pick_place',
        executable='cube_detector',
        name='cube_detector',
        parameters=[detector_config],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    pose_filter_node = Node(
        package='urxp_pick_place',
        executable='pose_filter',
        name='pose_filter',
        parameters=[detector_config],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # ── Vacuum gripper controller ───────────────────────────────────────────
    gripper_node = Node(
        package='urxp_pick_place',
        executable='gripper_node',
        name='urxp_gripper_node',
        parameters=[{'robot_ip': robot_ip}],
        output='screen',
    )

    # ── Pick-place server ────────────────────────────────────────────────────
    pick_place_node = Node(
        package='urxp_pick_place',
        executable='pick_place_server',
        name='urxp_pick_place_server',
        parameters=[pick_place_config],
        output='screen',
    )

    # ── Table marker for RViz context ───────────────────────────────────────
    table_node = Node(
        package='urxp_pick_place',
        executable='table_publisher',
        name='table_publisher',
        output='screen',
    )

    # Delay detector/filter startup to give RealSense time to stream.
    delayed_detector = TimerAction(
        period=6.0,
        actions=[cube_detector_node, pose_filter_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.165',
                               description='xArm Lite6 IP address (for gripper Tool GPIO)'),
        realsense_node,
        gripper_node,
        pick_place_node,
        table_node,
        delayed_detector,
    ])
