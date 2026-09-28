"""Launches the patrol state machine on its own, once sim + Nav2 +
greenhouse_vision are already up.

    ros2 launch greenhouse_route_manager route_manager.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="greenhouse_route_manager",
            executable="route_manager",
            name="route_manager",
            output="screen",
        ),
    ])
