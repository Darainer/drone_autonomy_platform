#!/usr/bin/env python3
"""Detector-agnostic multi-object tracker node.

Subscribes to ``vision_msgs/Detection2DArray`` from *any* upstream detector
(RT-DETR, YOLO, DETR, …) and maintains persistent track IDs using ByteTrack,
SORT, or BotSORT from the Roboflow ``supervision`` library.

When odometry is enabled, rotation-based ego-motion compensation is applied
before track association so that static objects don't drift when the drone
rotates between frames.  Both cuVSLAM (``nav_msgs/Odometry`` from Isaac ROS)
and MAVROS (``/mavros/local_position/odom``) are supported; the source is
selected at launch time via the ``odometry_source`` parameter.

Topics
------
Subscriptions:
  <detection_topic>               (vision_msgs/Detection2DArray)  — raw detections
  /visual_slam/tracking/odometry  (nav_msgs/Odometry)  — VSLAM pose  [optional]
  /mavros/local_position/odom     (nav_msgs/Odometry)  — MAVROS pose [optional]

Publications:
  /tracked_objects  (drone_autonomy_msgs/TrackedObjectArray)

Parameters
----------
tracker_type          : bytetrack | sort | botsort        (default: bytetrack)
detection_topic       : incoming detection topic           (default: /detections)
odometry_source       : vslam | mavros | none              (default: vslam)
vslam_odom_topic      : VSLAM odometry topic
mavros_odom_topic     : MAVROS odometry topic
camera_fx, camera_fy  : focal lengths [px] for ego-motion (default: OAK-D 1080p)
camera_cx, camera_cy  : principal point [px]
bt_track_activation_threshold  : ByteTrack min confidence to activate a track
bt_lost_track_buffer           : ByteTrack frames before lost track removal
bt_minimum_matching_threshold  : ByteTrack IoU threshold
bt_frame_rate                  : assumed frame rate for ByteTrack velocity model
bt_minimum_consecutive_frames  : ByteTrack frames before track is confirmed
sort_min_hits          : SORT min detections before track confirmed
sort_iou_threshold     : SORT IoU threshold
sort_max_age           : SORT frames before lost track removal
"""

from __future__ import annotations

import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from drone_autonomy_msgs.msg import TrackedObject, TrackedObjectArray

try:
    import supervision as sv
except ImportError as e:  # pragma: no cover
    raise SystemExit(
        "supervision package not found. Install with: pip install supervision"
    ) from e

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def _quat_to_rot(q) -> np.ndarray:
    """Convert a ROS quaternion message to a 3×3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _warp_boxes(xyxy: np.ndarray, H: np.ndarray) -> np.ndarray:
    """Apply a 3×3 homography to bounding-box centres, preserving box size.

    Args:
        xyxy: [N, 4] array of boxes in (x1, y1, x2, y2) format.
        H:    3×3 homography matrix.

    Returns:
        [N, 4] warped boxes (same size, shifted centres).
    """
    if len(xyxy) == 0:
        return xyxy

    cx = (xyxy[:, 0] + xyxy[:, 2]) / 2.0
    cy = (xyxy[:, 1] + xyxy[:, 3]) / 2.0
    w = xyxy[:, 2] - xyxy[:, 0]
    h = xyxy[:, 3] - xyxy[:, 1]

    pts = np.column_stack([cx, cy, np.ones(len(cx))])  # [N, 3]
    warped = (H @ pts.T).T  # [N, 3]
    cx_new = warped[:, 0] / warped[:, 2]
    cy_new = warped[:, 1] / warped[:, 2]

    return np.column_stack(
        [cx_new - w / 2, cy_new - h / 2, cx_new + w / 2, cy_new + h / 2]
    ).astype(np.float32)


class MultiObjectTrackerNode(Node):
    """ROS 2 node wrapping supervision trackers with optional odometry compensation."""

    def __init__(self) -> None:
        super().__init__("multi_object_tracker")

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter("tracker_type", "bytetrack")
        self.declare_parameter("detection_topic", "/detections")
        self.declare_parameter("odometry_source", "vslam")
        self.declare_parameter(
            "vslam_odom_topic", "/visual_slam/tracking/odometry"
        )
        self.declare_parameter(
            "mavros_odom_topic", "/mavros/local_position/odom"
        )
        # OAK-D 1080p approximate values (OV9782 @ 1920×1080)
        self.declare_parameter("camera_fx", 860.0)
        self.declare_parameter("camera_fy", 860.0)
        self.declare_parameter("camera_cx", 960.0)
        self.declare_parameter("camera_cy", 540.0)
        # ByteTrack
        self.declare_parameter("bt_track_activation_threshold", 0.25)
        self.declare_parameter("bt_lost_track_buffer", 30)
        self.declare_parameter("bt_minimum_matching_threshold", 0.8)
        self.declare_parameter("bt_frame_rate", 15)
        self.declare_parameter("bt_minimum_consecutive_frames", 1)
        # SORT
        self.declare_parameter("sort_min_hits", 3)
        self.declare_parameter("sort_iou_threshold", 0.3)
        self.declare_parameter("sort_max_age", 1)

        tracker_type: str = self.get_parameter("tracker_type").value
        odom_source: str = self.get_parameter("odometry_source").value

        # ── tracker ───────────────────────────────────────────────────────────
        self._tracker = self._create_tracker(tracker_type)
        self._tracker_type = tracker_type

        # ── class-ID registry (string label → int index for supervision) ─────
        self._class_to_idx: dict[str, int] = {}
        self._idx_to_class: dict[int, str] = {}

        # ── per-track bookkeeping ─────────────────────────────────────────────
        self._track_ages: dict[int, int] = {}
        self._track_last_pos: dict[int, tuple[float, float, float]] = {}

        # ── odometry state ────────────────────────────────────────────────────
        self._last_odom: Optional[Odometry] = None
        self._odom_at_prev_detection: Optional[Odometry] = None
        self._odom_source = odom_source

        # camera intrinsics for ego-motion compensation
        self._K = np.array(
            [
                [self.get_parameter("camera_fx").value, 0.0, self.get_parameter("camera_cx").value],
                [0.0, self.get_parameter("camera_fy").value, self.get_parameter("camera_cy").value],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self._K_inv = np.linalg.inv(self._K)

        # ── publisher ─────────────────────────────────────────────────────────
        self._tracked_pub = self.create_publisher(
            TrackedObjectArray, "/tracked_objects", 10
        )

        # ── detection subscriber ──────────────────────────────────────────────
        det_topic: str = self.get_parameter("detection_topic").value
        self._det_sub = self.create_subscription(
            Detection2DArray, det_topic, self._detection_cb, 10
        )

        # ── odometry subscriber (optional) ────────────────────────────────────
        if odom_source == "vslam":
            odom_topic: str = self.get_parameter("vslam_odom_topic").value
            self._odom_sub = self.create_subscription(
                Odometry, odom_topic, self._odom_cb, _SENSOR_QOS
            )
            self.get_logger().info(f"Ego-motion: VSLAM odometry from {odom_topic}")
        elif odom_source == "mavros":
            odom_topic = self.get_parameter("mavros_odom_topic").value
            self._odom_sub = self.create_subscription(
                Odometry, odom_topic, self._odom_cb, _SENSOR_QOS
            )
            self.get_logger().info(f"Ego-motion: MAVROS odometry from {odom_topic}")
        else:
            self._odom_sub = None
            self.get_logger().info("Ego-motion: disabled (odometry_source=none)")

        self.get_logger().info(
            f"Multi-object tracker ready | tracker={tracker_type} | odom={odom_source}"
        )

    # ── tracker factory ───────────────────────────────────────────────────────

    def _create_tracker(self, tracker_type: str):
        if tracker_type == "bytetrack":
            return sv.ByteTrack(
                track_activation_threshold=self.get_parameter(
                    "bt_track_activation_threshold"
                ).value,
                lost_track_buffer=self.get_parameter("bt_lost_track_buffer").value,
                minimum_matching_threshold=self.get_parameter(
                    "bt_minimum_matching_threshold"
                ).value,
                frame_rate=self.get_parameter("bt_frame_rate").value,
                minimum_consecutive_frames=self.get_parameter(
                    "bt_minimum_consecutive_frames"
                ).value,
            )
        elif tracker_type == "sort":
            return sv.SORT(
                min_hits=self.get_parameter("sort_min_hits").value,
                iou_threshold=self.get_parameter("sort_iou_threshold").value,
                max_age=self.get_parameter("sort_max_age").value,
            )
        elif tracker_type == "botsort":
            try:
                return sv.BotSORT()
            except AttributeError:
                self.get_logger().warn(
                    "BotSORT not available in this supervision version; "
                    "falling back to ByteTrack"
                )
                return sv.ByteTrack()
        else:
            self.get_logger().warn(
                f'Unknown tracker_type="{tracker_type}", using bytetrack'
            )
            return sv.ByteTrack()

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _detection_cb(self, msg: Detection2DArray) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        # ── convert Detection2DArray → supervision Detections ────────────────
        boxes: list[list[float]] = []
        scores: list[float] = []
        class_ids: list[int] = []

        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            boxes.append([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])

            score, class_str = 0.0, "0"
            if det.results:
                score = det.results[0].hypothesis.score
                class_str = det.results[0].hypothesis.class_id
            scores.append(score)
            class_ids.append(self._get_class_idx(class_str))

        if boxes:
            sv_dets = sv.Detections(
                xyxy=np.array(boxes, dtype=np.float32),
                confidence=np.array(scores, dtype=np.float32),
                class_id=np.array(class_ids, dtype=int),
            )
        else:
            sv_dets = sv.Detections.empty()

        # ── ego-motion compensation ───────────────────────────────────────────
        # Compute rotation homography H (old frame → new frame).
        # Warp detections by H⁻¹ so they align with the tracker's predictions
        # (which live in the previous camera orientation).  After tracking,
        # warp the output back by H to restore current-frame coordinates.
        H: Optional[np.ndarray] = self._compute_ego_homography()
        if H is not None and len(boxes) > 0:
            H_inv = np.linalg.inv(H)
            sv_dets.xyxy = _warp_boxes(sv_dets.xyxy, H_inv)

        # ── track ─────────────────────────────────────────────────────────────
        tracked = self._tracker.update_with_detections(sv_dets)

        # Restore current-frame coordinates after tracking
        if H is not None and tracked.xyxy is not None and len(tracked.xyxy) > 0:
            tracked.xyxy = _warp_boxes(tracked.xyxy, H)

        # ── build output message ──────────────────────────────────────────────
        out = TrackedObjectArray()
        out.header = msg.header
        out.tracker_type = self._tracker_type
        out.odometry_source = self._odom_source

        active_ids: set[int] = set()

        if tracked.tracker_id is not None:
            for i, tid in enumerate(tracked.tracker_id):
                tid = int(tid)
                if tid < 0:
                    continue

                active_ids.add(tid)

                x1, y1, x2, y2 = tracked.xyxy[i]
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                # Age
                self._track_ages[tid] = self._track_ages.get(tid, 0) + 1

                # Pixel velocity (pixels / second)
                vx, vy = 0.0, 0.0
                if tid in self._track_last_pos:
                    lx, ly, lt = self._track_last_pos[tid]
                    dt = now - lt
                    if dt > 0.0:
                        vx = (cx - lx) / dt
                        vy = (cy - ly) / dt
                self._track_last_pos[tid] = (cx, cy, now)

                obj = TrackedObject()
                obj.track_id = tid
                obj.class_id = self._idx_to_class.get(
                    int(tracked.class_id[i]) if tracked.class_id is not None else 0,
                    "unknown",
                )
                obj.score = float(
                    tracked.confidence[i] if tracked.confidence is not None else 0.0
                )
                obj.bbox.center.position.x = float(cx)
                obj.bbox.center.position.y = float(cy)
                obj.bbox.size_x = float(x2 - x1)
                obj.bbox.size_y = float(y2 - y1)
                obj.age = self._track_ages[tid]
                obj.velocity.x = vx
                obj.velocity.y = vy

                out.objects.append(obj)

        # Purge bookkeeping for tracks that have been dropped
        for stale_id in list(self._track_ages.keys()):
            if stale_id not in active_ids:
                self._track_ages.pop(stale_id, None)
                self._track_last_pos.pop(stale_id, None)

        self._tracked_pub.publish(out)

        # Save odometry snapshot for next frame's delta
        self._odom_at_prev_detection = self._last_odom

    # ── helpers ───────────────────────────────────────────────────────────────

    def _get_class_idx(self, class_str: str) -> int:
        if class_str not in self._class_to_idx:
            idx = len(self._class_to_idx)
            self._class_to_idx[class_str] = idx
            self._idx_to_class[idx] = class_str
        return self._class_to_idx[class_str]

    def _compute_ego_homography(self) -> Optional[np.ndarray]:
        """Return rotation homography H mapping previous frame → current frame.

        Uses the camera body orientation from odometry to estimate how a static
        scene point moves in pixel space due to drone rotation between frames.

        H = K · (R1ᵀ · R0) · K⁻¹

        where R0, R1 are camera-to-world rotation matrices at the previous and
        current detection frames respectively.  Translation compensation is
        omitted because it requires per-pixel depth.

        Returns None when insufficient odometry data is available.
        """
        if self._last_odom is None or self._odom_at_prev_detection is None:
            return None

        q0 = self._odom_at_prev_detection.pose.pose.orientation
        q1 = self._last_odom.pose.pose.orientation

        try:
            import tf_transformations

            R0 = tf_transformations.quaternion_matrix(
                [q0.x, q0.y, q0.z, q0.w]
            )[:3, :3]
            R1 = tf_transformations.quaternion_matrix(
                [q1.x, q1.y, q1.z, q1.w]
            )[:3, :3]
        except ImportError:
            R0 = _quat_to_rot(q0)
            R1 = _quat_to_rot(q1)

        # H maps p_old → p_new for a static scene point (rotation only)
        R_delta = R1.T @ R0  # camera's relative rotation
        H = self._K @ R_delta @ self._K_inv
        return H


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiObjectTrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
