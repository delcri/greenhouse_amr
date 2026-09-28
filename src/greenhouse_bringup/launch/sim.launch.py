"""Bring up the full simulation: Gazebo (greenhouse world) + robot spawn +
ros2_control + sensor/clock bridges + RViz.

    ros2 launch greenhouse_bringup sim.launch.py
    ros2 launch greenhouse_bringup sim.launch.py rviz:=false headless:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory("greenhouse_description")
    world_pkg = get_package_share_directory("greenhouse_world")
    bringup_pkg = get_package_share_directory("greenhouse_bringup")

    world_file = os.path.join(world_pkg, "worlds", "greenhouse.sdf")
    xacro_path = os.path.join(description_pkg, "urdf", "greenhouse_bot.urdf.xacro")
    bridge_config = os.path.join(bringup_pkg, "config", "gz_bridge.yaml")
    rviz_config = os.path.join(description_pkg, "rviz", "view_robot.rviz")

    use_rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")

    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    # GZ_SIM_RESOURCE_PATH so `model://grow_table` etc. resolve.
    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        os.path.join(world_pkg, "models")
        + os.pathsep
        + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [world_file, " -r"],
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "greenhouse_bot",
            # Spawn inside aisle 1 (y=1.5, see greenhouse.sdf), not at
            # y=0.0 -- that's row b's own y-coordinate, which put the
            # robot spawning nose-to-nose with a table instead of in
            # open aisle space.
            "-x", "-5.0", "-y", "1.5", "-z", "0.05",
        ],
        output="screen",
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
        output="screen",
    )

    camera_pan_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["camera_pan_controller"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # Spawn controllers only after the robot (and its ros2_control interfaces)
    # exist in sim, and one at a time: running spawner processes concurrently
    # races on controller_manager's switch_controller service (each spawner
    # does its own configure+activate call) and reliably makes one or more
    # controllers fail to activate. Chaining on_exit handlers serializes them.
    spawn_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    spawn_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )
    spawn_camera_pan = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[camera_pan_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),

        gz_sim,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        spawn_joint_state_broadcaster,
        spawn_diff_drive,
        spawn_camera_pan,
        rviz,
    ])
