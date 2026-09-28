"""Load the greenhouse_bot URDF and view it in RViz with joint_state_publisher_gui.

Useful as a fast sanity check on the model without needing Gazebo running.

    ros2 launch greenhouse_description view_robot.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("greenhouse_description")
    xacro_path = os.path.join(pkg_share, "urdf", "greenhouse_bot.urdf.xacro")
    rviz_config = os.path.join(pkg_share, "rviz", "view_robot.rviz")

    use_gui = LaunchConfiguration("use_gui")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui", default_value="true",
            description="Launch joint_state_publisher_gui to manually move joints"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
