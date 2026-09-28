"""Launches dock_target detection + the lateral alignment controller.

    ros2 launch greenhouse_vision vision.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="greenhouse_vision",
            executable="dock_target_detector",
            name="dock_target_detector",
            output="screen",
            parameters=[{
                "camera_frame": "camera_optical_link",
                "publish_debug_image": True,
            }],
        ),
        Node(
            package="greenhouse_vision",
            executable="table_alignment_controller",
            name="table_alignment_controller",
            output="screen",
            parameters=[{
                "base_frame": "base_link",
                "target_frame": "dock_target",
                "standoff_distance_m": 0.55,
            }],
        ),
    ])
