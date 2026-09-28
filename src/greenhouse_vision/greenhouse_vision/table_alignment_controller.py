#!/usr/bin/env python3
"""Visual-servo lateral alignment controller.

Looks up the dock_target's position (via TF, published by
dock_target_detector -- see that node's docstring for why there's no
marker ID involved) relative to base_link and drives a proportional
controller that:

  1. Waits while route_manager is sweeping the camera and searching.
  2. Once the target is tracked, nudges base_link laterally and forward
     until it sits at `standoff_distance_m` from the table -- the same
     "detect -> approach -> align -> commit" shape as pallet-face
     docking, applied to a table target instead of a fork pocket.

The aligned/not-aligned decision itself is delegated to
AlignmentEvaluator (Schmitt hysteresis + debounce, see that module),
matching the forklift project's alignment.py rather than a single fixed
threshold with a simple hold-counter.

On success it latches ALIGNED and stops publishing velocity commands so
route_manager can take over and log the reading.

Publishes:
    align_cmd_vel     (geometry_msgs/Twist)  velocity override while aligning
    alignment/status  (std_msgs/String)      SEARCHING | TRACKING | ALIGNED
Subscribes:
    ~/enable                        (std_msgs/Bool)    start/stop tracking (route_manager)
    dock_target_detector/confidence (std_msgs/Float32) 0..1, from the detector
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from greenhouse_vision.alignment_evaluator import AlignmentEvaluator, AlignmentParameters


class TableAlignmentController(Node):

    def __init__(self):
        super().__init__("table_alignment_controller")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("target_frame", "dock_target")
        self.declare_parameter("standoff_distance_m", 0.55)
        self.declare_parameter("lateral_tolerance_enter_m", 0.02)
        self.declare_parameter("lateral_tolerance_stay_m", 0.04)
        self.declare_parameter("distance_tolerance_enter_m", 0.03)
        self.declare_parameter("distance_tolerance_stay_m", 0.05)
        self.declare_parameter("debounce_frames", 5)
        self.declare_parameter("min_confidence", 0.3)
        self.declare_parameter("k_linear", 0.6)
        self.declare_parameter("k_angular", 1.4)
        self.declare_parameter("max_linear", 0.25)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("control_rate_hz", 10.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.standoff = self.get_parameter("standoff_distance_m").value
        self.min_confidence = self.get_parameter("min_confidence").value
        self.k_linear = self.get_parameter("k_linear").value
        self.k_angular = self.get_parameter("k_angular").value
        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value

        self.evaluator = AlignmentEvaluator(AlignmentParameters(
            lateral_tol_enter=self.get_parameter("lateral_tolerance_enter_m").value,
            lateral_tol_stay=self.get_parameter("lateral_tolerance_stay_m").value,
            distance_tol_enter=self.get_parameter("distance_tolerance_enter_m").value,
            distance_tol_stay=self.get_parameter("distance_tolerance_stay_m").value,
            debounce_frames=self.get_parameter("debounce_frames").value,
        ))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.enabled = False
        self.confidence = 0.0
        self.status = "SEARCHING"

        self.create_subscription(Bool, "~/enable", self._on_enable, 10)
        self.create_subscription(
            Float32, "dock_target_detector/confidence", self._on_confidence, 10
        )

        self.cmd_pub = self.create_publisher(Twist, "align_cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "alignment/status", 10)

        rate = self.get_parameter("control_rate_hz").value
        self.create_timer(1.0 / rate, self._control_step)

        self.get_logger().info("table_alignment_controller ready; awaiting ~/enable")

    def _on_enable(self, msg: Bool):
        self.enabled = msg.data
        if self.enabled:
            self.evaluator.reset()
            self._set_status("SEARCHING")
        else:
            self.cmd_pub.publish(Twist())

    def _on_confidence(self, msg: Float32):
        self.confidence = msg.data

    def _control_step(self):
        if not self.enabled:
            return

        if self.confidence < self.min_confidence:
            self._set_status("SEARCHING")
            self.cmd_pub.publish(Twist())  # hold still / let route_manager decide fallback
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.target_frame, rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self._set_status("SEARCHING")
            self.cmd_pub.publish(Twist())
            return

        self._set_status("TRACKING")

        # In base_link, +x is forward, +y is left. We want to end up
        # `standoff` meters short of the target along x, and zeroed in y.
        distance_error = tf.transform.translation.x - self.standoff
        lateral_error = tf.transform.translation.y

        result = self.evaluator.evaluate(lateral_error, distance_error, self.confidence)

        if result.aligned:
            self._set_status("ALIGNED")
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()
        # Heading correction dominates when lateral error is large (servo the
        # nose toward the target first), forward speed ramps in as it settles.
        heading_error = math.atan2(lateral_error, max(distance_error, 0.05))

        cmd.linear.x = self._clamp(self.k_linear * distance_error, self.max_linear)
        cmd.angular.z = self._clamp(self.k_angular * heading_error, self.max_angular)

        self.cmd_pub.publish(cmd)

    def _set_status(self, status: str):
        if status != self.status:
            self.status = status
            self.status_pub.publish(String(data=status))

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = TableAlignmentController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
