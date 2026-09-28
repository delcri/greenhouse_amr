#!/usr/bin/env python3
"""Top-level state machine for the greenhouse AMR.

Ties Nav2 route-following, the vision alignment controller
(greenhouse_vision) and a stub reading log together into a single
patrol cycle, one grow table at a time:

    IDLE -> TRAVEL -> SEARCH -> ALIGN -> LOG -> RESUME -> (next waypoint)

This is the same "detect -> approach -> align -> commit -> resume" shape
used for pallet pick/place docking, applied here to a table-reading task
so the pattern is demonstrated on non-proprietary hardware/world.

- TRAVEL sends a Nav2 NavigateToPose goal to get abreast of the next
  table and waits for the action result.
- SEARCH sweeps the camera pan joint and enables table_alignment_controller
  (dock_target_detector doesn't decode a per-table ID -- see its
  docstring -- so there's nothing to tell it *which* target to track,
  just when to start looking); once it reports TRACKING we move on.
- ALIGN forwards table_alignment_controller's `align_cmd_vel` straight to
  the base controller (a minimal hand-rolled cmd_vel mux -- Nav2 does not
  publish cmd_vel while idle between goals, so this is safe) until the
  controller reports ALIGNED.
- LOG publishes a stub reading for the table (its identity comes from
  the waypoint route_manager is already at, not from vision) and moves
  to the next stop.

Waypoints come from greenhouse_navigation/config/aisle_route.yaml.
"""
import time
from enum import Enum, auto

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String, Float64MultiArray

try:
    from tf_transformations import quaternion_from_euler
except ImportError:  # pragma: no cover - fallback if tf_transformations isn't installed
    import math

    def quaternion_from_euler(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )


class State(Enum):
    IDLE = auto()
    TRAVEL = auto()
    SEARCH = auto()
    ALIGN = auto()
    LOG = auto()
    RESUME = auto()
    DONE = auto()


class RouteManager(Node):

    SEARCH_TIMEOUT_S = 12.0
    SEARCH_SWEEP_POSITIONS = [-0.5, 0.0, 0.5, 0.0]
    SEARCH_SWEEP_PERIOD_S = 2.5

    def __init__(self):
        super().__init__("route_manager")

        self.declare_parameter("route_file", "")
        self.declare_parameter("nav_action_name", "navigate_to_pose")

        route_file = self.get_parameter("route_file").value or self._default_route_path()
        self.route = self._load_route(route_file)
        self.route_index = 0

        self.state = State.IDLE
        self._nav_goal_handle = None
        self._search_started_at = None
        self._search_sweep_i = 0
        self._search_sweep_last_change = 0.0

        cb_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter("nav_action_name").value,
            callback_group=cb_group,
        )

        self.enable_pub = self.create_publisher(Bool, "table_alignment_controller/enable", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "diff_drive_base_controller/cmd_vel_unstamped", 10)
        self.pan_cmd_pub = self.create_publisher(Float64MultiArray, "camera_pan_controller/commands", 10)
        self.readings_pub = self.create_publisher(String, "table_readings", 10)
        self.state_pub = self.create_publisher(String, "route_manager/state", 10)

        self.create_subscription(String, "alignment/status", self._on_alignment_status, 10)
        self.create_subscription(Twist, "align_cmd_vel", self._on_align_cmd_vel, 10)

        self._alignment_status = "SEARCHING"

        self.create_timer(0.2, self._tick)
        self.get_logger().info(f"route_manager loaded {len(self.route)} waypoints, starting patrol")
        self._set_state(State.TRAVEL)

    # ------------------------------------------------------------------ #
    # route loading
    # ------------------------------------------------------------------ #
    @staticmethod
    def _default_route_path():
        pkg_share = get_package_share_directory("greenhouse_navigation")
        return f"{pkg_share}/config/aisle_route.yaml"

    def _load_route(self, path):
        with open(path) as f:
            data = yaml.safe_load(f)
        return data["waypoints"]

    # ------------------------------------------------------------------ #
    # state machine driver
    # ------------------------------------------------------------------ #
    def _set_state(self, new_state: State):
        if new_state != self.state:
            self.get_logger().info(f"[state] {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_pub.publish(String(data=new_state.name))

        if new_state == State.TRAVEL:
            self._send_nav_goal()
        elif new_state == State.SEARCH:
            self._enter_search()
        elif new_state == State.LOG:
            self._log_reading()
        elif new_state == State.RESUME:
            self._resume()
        elif new_state == State.DONE:
            self.get_logger().info("patrol complete")

    def _tick(self):
        if self.state == State.SEARCH:
            self._search_step()
        # TRAVEL/ALIGN/LOG/RESUME/DONE are event-driven (action callbacks,
        # topic callbacks, or one-shot on entry) rather than polled here.

    # ------------------------------------------------------------------ #
    # TRAVEL
    # ------------------------------------------------------------------ #
    def _send_nav_goal(self):
        waypoint = self.route[self.route_index]
        pose = waypoint["pose"]

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose["x"])
        goal.pose.pose.position.y = float(pose["y"])
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(pose["yaw"]))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"traveling to '{waypoint['name']}' ({pose['x']}, {pose['y']}, yaw={pose['yaw']})"
        )

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("nav2 action server not up yet, will retry on next TRAVEL entry")
            self.create_timer(2.0, self._retry_send_nav_goal_once)
            return

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_goal_response)

    def _retry_send_nav_goal_once(self):
        if self.state == State.TRAVEL:
            self._send_nav_goal()

    def _on_nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("nav2 rejected the goal, skipping this waypoint")
            self._set_state(State.RESUME)
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        status = future.result().status
        # status 4 == STATUS_SUCCEEDED in action_msgs/msg/GoalStatus
        if status == 4:
            self._set_state(State.SEARCH)
        else:
            self.get_logger().warn(f"nav2 goal did not succeed (status={status}), skipping table")
            self._set_state(State.RESUME)

    # ------------------------------------------------------------------ #
    # SEARCH
    # ------------------------------------------------------------------ #
    def _enter_search(self):
        self.enable_pub.publish(Bool(data=True))
        self._search_started_at = time.monotonic()
        self._search_sweep_i = 0
        self._search_sweep_last_change = time.monotonic()
        self._alignment_status = "SEARCHING"

    def _search_step(self):
        now = time.monotonic()

        # Sweep the camera while waiting for a track.
        if now - self._search_sweep_last_change > self.SEARCH_SWEEP_PERIOD_S:
            self._search_sweep_i = (self._search_sweep_i + 1) % len(self.SEARCH_SWEEP_POSITIONS)
            self.pan_cmd_pub.publish(
                Float64MultiArray(data=[self.SEARCH_SWEEP_POSITIONS[self._search_sweep_i]])
            )
            self._search_sweep_last_change = now

        if self._alignment_status in ("TRACKING", "ALIGNED"):
            self.pan_cmd_pub.publish(Float64MultiArray(data=[0.0]))  # re-center for the approach
            self._set_state(State.ALIGN)
            return

        if now - self._search_started_at > self.SEARCH_TIMEOUT_S:
            waypoint = self.route[self.route_index]
            self.get_logger().warn(f"timed out searching for a target at '{waypoint['name']}'")
            self.pan_cmd_pub.publish(Float64MultiArray(data=[0.0]))
            self._set_state(State.RESUME)

    # ------------------------------------------------------------------ #
    # ALIGN  (forwards table_alignment_controller's output to cmd_vel)
    # ------------------------------------------------------------------ #
    def _on_align_cmd_vel(self, msg: Twist):
        if self.state == State.ALIGN:
            self.cmd_vel_pub.publish(msg)

    def _on_alignment_status(self, msg: String):
        self._alignment_status = msg.data
        if self.state == State.ALIGN and msg.data == "ALIGNED":
            self.cmd_vel_pub.publish(Twist())  # explicit stop
            self._set_state(State.LOG)
        elif self.state == State.ALIGN and msg.data == "SEARCHING":
            # lost the marker mid-approach; back off to SEARCH rather than
            # keep forwarding a stale/garbage velocity command
            self._set_state(State.SEARCH)

    # ------------------------------------------------------------------ #
    # LOG / RESUME
    # ------------------------------------------------------------------ #
    def _log_reading(self):
        waypoint = self.route[self.route_index]
        # Stub sensor reading -- stands in for whatever payload sensor a
        # real deployment would read at the dock (soil moisture, a barcode
        # scan, a weight cell, ...). The point being demonstrated here is
        # the docking + handoff, not the sensor itself.
        reading = {
            "table": waypoint["name"],
            "marker_id": waypoint["marker_id"],
            "stamp": self.get_clock().now().to_msg().sec,
        }
        self.readings_pub.publish(String(data=str(reading)))
        self.get_logger().info(f"logged reading at '{waypoint['name']}': {reading}")
        self._set_state(State.RESUME)

    def _resume(self):
        self.route_index += 1
        self.enable_pub.publish(Bool(data=False))
        if self.route_index >= len(self.route):
            self._set_state(State.DONE)
        else:
            self._set_state(State.TRAVEL)


def main(args=None):
    rclpy.init(args=args)
    node = RouteManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
