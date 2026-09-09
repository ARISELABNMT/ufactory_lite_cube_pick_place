"""
URXP robot bringup — MoveIt + xArm Lite6 driver + RViz (+ vacuum gripper, RealSense D435i mount).

Terminal 1 of the URXP pick-place workflow:
  source /opt/ros/jazzy/setup.bash
  source ~/URXP_ws/install/setup.bash
  ros2 launch urxp_pick_place urxp_robot.launch.py robot_ip:=192.168.1.165
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper')

    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_realmove.launch.py'
        ])),
        launch_arguments={
            'robot_ip': robot_ip,
            'dof': '6',
            'robot_type': 'lite',
            'hw_ns': 'ufactory',
            'no_gui_ctrl': 'false',
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_realsense_d435i': 'true',
            'add_d435i_links': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.165',
                               description='xArm Lite6 IP address'),
        DeclareLaunchArgument('add_vacuum_gripper', default_value='true',
                               description='Add vacuum gripper to robot model'),
        moveit_stack,
    ])
