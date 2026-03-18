#!/usr/bin/env python3
"""RF-DETR inference node — subscribes to OAK-D RGB frames and publishes detections."""

from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
)


def _imgmsg_to_rgb(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image to an (H, W, 3) uint8 RGB array.

    Uses ``msg.step`` (bytes per row) so that row-padded frames — where
    ``msg.step > msg.width * channels`` — are decoded correctly.  Ignoring
    ``msg.step`` and reshaping directly to ``(height, width, 3)`` works only
    when rows are tightly packed and will raise or silently mis-index pixels
    when any padding is present.
    """
    enc = msg.encoding.lower()
    if enc in ("rgb8",):
        channels = 3
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = arr[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        return img
    if enc in ("bgr8",):
        channels = 3
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = arr[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        return img[:, :, ::-1]  # BGR → RGB
    # Fall back to cv_bridge for mono, bayer, 16-bit, compressed, etc.
    bridge = CvBridge()
    bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    return bgr[:, :, ::-1]


class RfDetrNode(Node):
    def __init__(self) -> None:
        super().__init__("rfdetr_node")

        self.declare_parameter("confidence_threshold", 0.5)
        self._conf_thresh: float = (
            self.get_parameter("confidence_threshold").get_parameter_value().double_value
        )

        self._bridge = CvBridge()

        self._sub = self.create_subscription(
            Image,
            "/oak/rgb/image_raw",
            self._image_callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._pub = self.create_publisher(Detection2DArray, "/detections", 10)

        self.get_logger().info(
            f"RF-DETR node ready — listening on /oak/rgb/image_raw "
            f"(conf ≥ {self._conf_thresh})"
        )

    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image) -> None:
        try:
            rgb = _imgmsg_to_rgb(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Image decode failed: {exc}")
            return

        detections = self._run_inference(rgb)
        self._pub.publish(self._build_msg(msg, detections))

    # ------------------------------------------------------------------
    def _run_inference(
        self, rgb: np.ndarray
    ) -> list[tuple[float, float, float, float, str, float]]:
        """Run RF-DETR inference and return a list of (cx, cy, w, h, class_id, score)."""
        # TODO: load TensorRT engine / ONNX model and replace this stub.
        return []

    # ------------------------------------------------------------------
    def _build_msg(
        self,
        source: Image,
        raw: list[tuple[float, float, float, float, str, float]],
    ) -> Detection2DArray:
        out = Detection2DArray()
        out.header = source.header
        for cx, cy, w, h, class_id, score in raw:
            if score < self._conf_thresh:
                continue
            det = Detection2D()
            det.header = source.header
            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_id)
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out.detections.append(det)
        return out


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RfDetrNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
