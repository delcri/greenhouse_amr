"""RTAB-Map visual SLAM off the mast RGB-D camera.

No lidar on this robot -- mapping, localization and (via
greenhouse_navigation's costmaps) obstacle avoidance are all camera-only,
mirroring the forklift project's sensing approach (RealSense/ZED + RTAB-Map
instead of a dedicated lidar). Three-node pipeline, the standard rtabmap_ros
shape:

    camera/{image_raw,depth_image,camera_info}
        -> rgbd_sync         (rtabmap_sync)    combines into one rgbd_image
        -> rgbd_odometry     (rtabmap_odom)    visual odometry: odom -> base_link
        -> rtabmap           (rtabmap_slam)    SLAM: map -> odom, /map, /grid_map
        -> rtabmap_viz       (rtabmap_viz)     GUI: live map, graph, loop closures

Run alongside sim.launch.py:

    ros2 launch greenhouse_vslam rtabmap.launch.py
    ros2 launch greenhouse_vslam rtabmap.launch.py database_path:=~/.ros/greenhouse.db
    ros2 launch greenhouse_vslam rtabmap.launch.py viz:=false   # headless, no GUI window
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")
    use_viz = LaunchConfiguration("viz")

    rgb_topic = "camera/image_raw"
    depth_topic = "camera/depth_image"
    camera_info_topic = "camera/camera_info"

    sync_remappings = [
        ("rgb/image", rgb_topic),
        ("rgb/camera_info", camera_info_topic),
        ("depth/image", depth_topic),
    ]

    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        output="screen",
        parameters=[{
            "approx_sync": True,
            "use_sim_time": use_sim_time,
        }],
        remappings=sync_remappings,
    )

    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "publish_tf": True,
            "subscribe_rgbd": True,
            "approx_sync": True,
            "wait_imu_to_init": False,
            "use_sim_time": use_sim_time,
            # ground robot: constrain odometry to 3DoF (x, y, yaw)
            "Reg/Force3DoF": "true",
            "Odom/Holonomic": "false",
            "Vis/MinInliers": "15",
        }],
        # rgbd_image comes from rgbd_sync above (default topic name).
        # Renamed to visual_odom to avoid colliding with the wheel odometry
        # diff_drive_base_controller publishes on the default /odom topic.
        remappings=[("odom", "visual_odom")],
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "publish_tf": True,
            "subscribe_rgbd": True,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "use_sim_time": use_sim_time,
            "database_path": database_path,

            "Reg/Force3DoF": "true",
            # 2D occupancy grid (not a 3D voxel map) -- this is what feeds
            # Nav2's global costmap static_layer via /map.
            "Grid/3D": "false",
            "Grid/RangeMax": "12.0",
            "Grid/RayTracing": "true",
            "RGBD/AngularUpdate": "0.05",
            "RGBD/LinearUpdate": "0.05",
        }],
        remappings=[("odom", "visual_odom")],
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        condition=IfCondition(use_viz),
        parameters=[{
            "subscribe_rgbd": True,
            "approx_sync": True,
            "use_sim_time": use_sim_time,
            "frame_id": "base_link",
            "odom_frame_id": "odom",
        }],
        # Same topic wiring as the rtabmap node above: rgbd_image from
        # rgbd_sync (default topic name), odom from rgbd_odometry's
        # renamed output.
        remappings=[("odom", "visual_odom")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "database_path", default_value="~/.ros/greenhouse_rtabmap.db",
            description="Where RTAB-Map's map database is saved/loaded from."
        ),
        DeclareLaunchArgument(
            "viz", default_value="true",
            description="Launch rtabmap_viz (the live map/graph GUI window)."
        ),

        rgbd_sync,
        rgbd_odometry,
        rtabmap,
        rtabmap_viz,
    ])
