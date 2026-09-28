#!/usr/bin/env python3
"""Detects the dock_target plate (a distinctive-color patch, no coded ID)
in the mast camera feed and estimates its 3D position via classical
color/contour detection + per-point depth unprojection.

This mirrors the forklift project's depth-based alignment pipeline
(vision_core/alignment_error.py) rather than a fiducial-marker approach:
a 2D detection (there: YOLO segmentation of pallet/fork-holes; here: an
HSV color threshold) gives a pixel location, which gets unprojected to a
metric point by sampling the depth image AT that pixel (median over a
small window, for robustness) rather than assuming a single distance for
the whole object -- the same reasoning their docstring gives for why
per-point depth matters over "just use the pallet's mean depth". An EMA
smooths the result frame to frame.

There is deliberately no marker ID anywhere in this node: which grow
table is being approached is context route_manager already has from the
Nav2 waypoint it just drove to, not something decoded from the image --
matching how the forklift's vision pipeline doesn't identify *which*
pallet either.

Published:
    ~/confidence   (std_msgs/Float32)   0..1, decays to 0 when no detection
    ~/debug_image  (sensor_msgs/Image)  annotated image for RViz/rqt
    tf: camera_optical_link -> dock_target   only while confidence > 0
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import message_filters

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster


class DockTargetDetector(Node):

    def __init__(self):
        super().__init__("dock_target_detector")

        # HSV range for the safety-orange dock_target plate (see
        # greenhouse_world/models/dock_target). Wide-ish on purpose --
        # Gazebo's flat-shaded lighting doesn't add much texture noise,
        # but ambient/diffuse balance still shifts saturation/value a bit
        # depending on the sun angle.
        self.declare_parameter("hsv_lower", [5, 120, 100])
        self.declare_parameter("hsv_upper", [25, 255, 255])
        self.declare_parameter("min_contour_area_px", 150)
        self.declare_parameter("depth_sample_radius_px", 2)  # 5x5 median window
        self.declare_parameter("ema_alpha", 0.35)
        self.declare_parameter("camera_frame", "camera_optical_link")
        self.declare_parameter("publish_debug_image", True)

        self.hsv_lower = np.array(self.get_parameter("hsv_lower").value, dtype=np.uint8)
        self.hsv_upper = np.array(self.get_parameter("hsv_upper").value, dtype=np.uint8)
        self.min_area = self.get_parameter("min_contour_area_px").value
        self.depth_radius = self.get_parameter("depth_sample_radius_px").value
        self.alpha = self.get_parameter("ema_alpha").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.publish_debug = self.get_parameter("publish_debug_image").value

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_matrix = None
        self._smoothed_point = None   # EMA state: (x, y, z) in camera_optical_link
        self._confidence = 0.0

        self.create_subscription(
            CameraInfo, "camera/camera_info", self._on_camera_info, qos_profile_sensor_data
        )

        rgb_sub = message_filters.Subscriber(
            self, Image, "camera/image_raw", qos_profile=qos_profile_sensor_data
        )
        depth_sub = message_filters.Subscriber(
            self, Image, "camera/depth_image", qos_profile=qos_profile_sensor_data
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._on_rgbd)

        self.confidence_pub = self.create_publisher(Float32, "~/confidence", 10)
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, "~/debug_image", 10)

        self.get_logger().info(
            "dock_target_detector ready, waiting for camera_info + synced rgb/depth"
        )

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image):
        if self.camera_matrix is None:
            return  # no calibration yet

        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        centroid_px, area, contour = self._find_target(frame)

        if centroid_px is None:
            self._decay_confidence()
            self._publish_debug(frame, None, None)
            return

        depth_m = self._sample_depth(depth, centroid_px)
        if depth_m is None or depth_m <= 0.0 or not np.isfinite(depth_m):
            self._decay_confidence()
            self._publish_debug(frame, centroid_px, None)
            return

        point_cam = self._unproject(centroid_px, depth_m)
        self._smoothed_point = self._ema(self._smoothed_point, point_cam)

        # Confidence: normalized contour area, clipped -- a bigger, more
        # solid blob is a more trustworthy detection. Rises immediately on
        # a good frame, decays gradually when frames are missed (handled
        # in _decay_confidence), same shape as their "confidence drops to
        # 0 when the signal drops out" behavior.
        self._confidence = float(np.clip(area / 3000.0, 0.0, 1.0))

        self._broadcast_tf(rgb_msg, self._smoothed_point)
        self.confidence_pub.publish(Float32(data=self._confidence))
        self._publish_debug(frame, centroid_px, depth_m, contour)

    def _find_target(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0.0, None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_area:
            return None, 0.0, None

        m = cv2.moments(largest)
        if m["m00"] == 0:
            return None, 0.0, None
        cx = m["m10"] / m["m00"]
        cy = m["m01"] / m["m00"]
        return (cx, cy), area, largest

    def _sample_depth(self, depth_img, centroid_px):
        u, v = int(round(centroid_px[0])), int(round(centroid_px[1]))
        r = self.depth_radius
        h, w = depth_img.shape[:2]
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        window = np.asarray(depth_img[v0:v1, u0:u1], dtype=np.float64).flatten()
        window = window[np.isfinite(window) & (window > 0.0)]
        if window.size == 0:
            return None
        return float(np.median(window))

    def _unproject(self, centroid_px, depth_m):
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx0, cy0 = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        u, v = centroid_px
        # REP-103 optical frame: x right, y down, z forward.
        x = (u - cx0) * depth_m / fx
        y = (v - cy0) * depth_m / fy
        z = depth_m
        return np.array([x, y, z])

    def _ema(self, prev, new):
        if prev is None:
            return new
        return self.alpha * new + (1.0 - self.alpha) * prev

    def _decay_confidence(self, factor=0.7):
        self._confidence *= factor
        if self._confidence < 0.05:
            self._confidence = 0.0
            self._smoothed_point = None  # stale -- don't keep broadcasting a frozen TF
        self.confidence_pub.publish(Float32(data=self._confidence))

    def _broadcast_tf(self, msg: Image, point_cam):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = "dock_target"
        t.transform.translation.x = float(point_cam[0])
        t.transform.translation.y = float(point_cam[1])
        t.transform.translation.z = float(point_cam[2])
        t.transform.rotation.w = 1.0  # no orientation estimate from a color blob
        self.tf_broadcaster.sendTransform(t)

    def _publish_debug(self, frame, centroid_px, depth_m, contour=None):
        if not self.publish_debug:
            return
        vis = frame.copy()
        if contour is not None:
            cv2.drawContours(vis, [contour], -1, (0, 255, 0), 2)
        if centroid_px is not None:
            u, v = int(centroid_px[0]), int(centroid_px[1])
            cv2.drawMarker(vis, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
            label = f"conf={self._confidence:.2f}"
            if depth_m is not None:
                label += f" z={depth_m:.2f}m"
            cv2.putText(vis, label, (u + 10, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DockTargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
