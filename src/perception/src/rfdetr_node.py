#!/usr/bin/env python3
"""RF-DETR-S TensorRT FP16 inference node for ROS2.

Subscribes to camera images, runs RF-DETR-S object detection via TensorRT,
and publishes vision_msgs/Detection2DArray on /detections.

RF-DETR-S is NMS-free (end-to-end), Apache 2.0 licensed, and uses a DINOv2
backbone. Input: 512x512, Output: top-300 detections (labels, boxes, scores).

Designed for NVIDIA Jetson Orin Nano with TensorRT FP16.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from std_msgs.msg import Header

# COCO 2017 class names (80 classes) — RF-DETR uses standard COCO class IDs
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


class TensorRTEngine:
    """Manages a TensorRT engine for RF-DETR-S inference."""

    def __init__(self, engine_path: str, logger):
        import tensorrt as trt
        import pycuda.autoinit  # noqa: F401 — initializes CUDA context
        import pycuda.driver as cuda

        self._cuda = cuda
        self._logger = logger

        trt_logger = trt.Logger(trt.Logger.WARNING)
        logger.info(f"Loading TensorRT engine: {engine_path}")

        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(trt_logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        # Allocate host/device buffers for all bindings
        self.inputs = {}
        self.outputs = {}
        self._d_buffers = []
        self._bindings = []

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = self.engine.get_tensor_shape(name)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            size = int(np.prod(shape))
            host_buf = np.empty(shape, dtype=dtype)
            dev_buf = cuda.mem_alloc(host_buf.nbytes)

            self._bindings.append(int(dev_buf))
            self.context.set_tensor_address(name, int(dev_buf))

            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.inputs[name] = {"host": host_buf, "device": dev_buf, "shape": shape}
                logger.info(f"  Input  '{name}': {shape} ({dtype.__name__})")
            else:
                self.outputs[name] = {"host": host_buf, "device": dev_buf, "shape": shape}
                logger.info(f"  Output '{name}': {shape} ({dtype.__name__})")

        self.stream = cuda.Stream()
        logger.info("TensorRT engine ready.")

    def infer(self, image_tensor: np.ndarray) -> dict:
        """Run inference on a preprocessed image tensor (1,3,512,512) float32."""
        cuda = self._cuda

        # Copy input to device
        inp = self.inputs["images"]
        np.copyto(inp["host"], image_tensor)
        cuda.memcpy_htod_async(inp["device"], inp["host"], self.stream)

        # Execute
        self.context.execute_async_v3(stream_handle=self.stream.handle)

        # Copy outputs back
        results = {}
        for name, out in self.outputs.items():
            cuda.memcpy_dtoh_async(out["host"], out["device"], self.stream)
            results[name] = out["host"]

        self.stream.synchronize()
        return results


class RFDETRNode(Node):
    """ROS2 node running RF-DETR-S object detection via TensorRT FP16."""

    # RF-DETR-S input resolution
    INPUT_H = 512
    INPUT_W = 512

    # ImageNet normalization (used by DINOv2 backbone)
    MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    def __init__(self):
        super().__init__("rfdetr_node")

        # Parameters
        self.declare_parameter("engine_path", "/home/dev/models/rfdetr_s_coco.engine")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("max_detections", 100)

        engine_path = self.get_parameter("engine_path").value
        self.conf_threshold = self.get_parameter("confidence_threshold").value
        image_topic = self.get_parameter("image_topic").value
        self.max_detections = self.get_parameter("max_detections").value

        self.get_logger().info(f"RF-DETR-S node starting (conf={self.conf_threshold})")
        self.get_logger().info(f"  Engine: {engine_path}")
        self.get_logger().info(f"  Input:  {self.INPUT_W}x{self.INPUT_H} FP16")

        # Load TensorRT engine
        self.trt_engine = TensorRTEngine(engine_path, self.get_logger())

        # Publisher — same topic as the old RT-DETR pipeline for drop-in compat
        self.det_pub = self.create_publisher(Detection2DArray, "/detections", 10)

        # Subscriber — sensor data QoS for camera
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, sensor_qos
        )

        # Track inference rate
        self._frame_count = 0
        self._timer = self.create_timer(10.0, self._log_stats)

        self.get_logger().info("RF-DETR-S node ready — waiting for images")

    def image_callback(self, msg: Image):
        """Process incoming camera frame through RF-DETR-S."""
        # Convert ROS Image to numpy (handle both rgb8 and bgr8)
        img = self._ros_image_to_numpy(msg)
        if img is None:
            return

        # Preprocess: resize, normalize, NCHW
        input_tensor = self._preprocess(img)

        # TensorRT inference
        outputs = self.trt_engine.infer(input_tensor)

        # Parse detections and publish
        det_msg = self._postprocess(outputs, msg.header, img.shape[1], img.shape[0])
        self.det_pub.publish(det_msg)
        self._frame_count += 1

    def _ros_image_to_numpy(self, msg: Image) -> np.ndarray | None:
        """Convert sensor_msgs/Image to HWC uint8 RGB numpy array."""
        if msg.encoding in ("rgb8", "RGB8"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding in ("bgr8", "BGR8"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img = img[:, :, ::-1]  # BGR -> RGB
        else:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}", throttle_duration_sec=5.0)
            return None
        return img

    def _preprocess(self, img: np.ndarray) -> np.ndarray:
        """Resize, normalize, and convert to NCHW float32 tensor."""
        import cv2

        # Resize to 512x512 (RF-DETR-S input)
        resized = cv2.resize(img, (self.INPUT_W, self.INPUT_H), interpolation=cv2.INTER_LINEAR)

        # Normalize: [0,255] -> [0,1] -> ImageNet normalize
        tensor = resized.astype(np.float32) / 255.0
        tensor = (tensor - self.MEAN) / self.STD

        # HWC -> NCHW
        tensor = np.transpose(tensor, (2, 0, 1))[np.newaxis, ...]
        return np.ascontiguousarray(tensor)

    def _postprocess(
        self, outputs: dict, header: Header, orig_w: int, orig_h: int
    ) -> Detection2DArray:
        """Convert RF-DETR outputs to Detection2DArray.

        RF-DETR-S outputs (NMS-free, top-300):
            labels: (1, 300) int64 — class indices
            boxes:  (1, 300, 4) float32 — [cx, cy, w, h] normalized to [0,1]
            scores: (1, 300) float32 — confidence scores
        """
        labels = outputs["labels"].flatten()
        scores = outputs["scores"].flatten()
        boxes = outputs["boxes"].reshape(-1, 4)

        det_array = Detection2DArray()
        det_array.header = header

        # Filter by confidence and limit count
        mask = scores >= self.conf_threshold
        indices = np.where(mask)[0]

        # Sort by score descending, take top-N
        if len(indices) > self.max_detections:
            top_idx = np.argsort(scores[indices])[::-1][: self.max_detections]
            indices = indices[top_idx]

        for idx in indices:
            det = Detection2D()
            det.header = header

            # RF-DETR outputs normalized [cx, cy, w, h] — scale to pixel coords
            cx, cy, w, h = boxes[idx]
            det.bbox.center.position.x = float(cx * orig_w)
            det.bbox.center.position.y = float(cy * orig_h)
            det.bbox.size_x = float(w * orig_w)
            det.bbox.size_y = float(h * orig_h)

            hyp = ObjectHypothesisWithPose()
            class_id = int(labels[idx])
            hyp.hypothesis.class_id = str(class_id)
            hyp.hypothesis.score = float(scores[idx])
            det.results.append(hyp)

            det_array.detections.append(det)

        return det_array

    def _log_stats(self):
        """Log inference throughput every 10 seconds."""
        if self._frame_count > 0:
            self.get_logger().info(
                f"RF-DETR-S: {self._frame_count} frames / 10s "
                f"({self._frame_count / 10.0:.1f} FPS)"
            )
            self._frame_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = RFDETRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
