#!/usr/bin/env python3
"""Draw RT-DETR bounding boxes on the OAK-D RGB stream and republish."""

import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

# Colour per class index (cycles if more classes than entries)
_PALETTE = [
    (0, 255, 0),
    (255, 80, 80),
    (80, 80, 255),
    (255, 200, 0),
    (0, 200, 255),
    (200, 0, 255),
]


class DetectionVisualizer(Node):
    def __init__(self):
        super().__init__('detection_visualizer')
        self.bridge = CvBridge()

        img_sub = message_filters.Subscriber(self, Image, '/oak/rgb/image_raw')
        det_sub = message_filters.Subscriber(self, Detection2DArray, '/detections')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, det_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self._callback)

        self.pub = self.create_publisher(Image, '/visualization/detection_overlay', 10)
        self.get_logger().info('Detection visualizer ready — publishing /visualization/detection_overlay')

    def _callback(self, img_msg: Image, det_msg: Detection2DArray):
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        for det in det_msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            x1 = int(cx - det.bbox.size_x / 2)
            y1 = int(cy - det.bbox.size_y / 2)
            x2 = int(cx + det.bbox.size_x / 2)
            y2 = int(cy + det.bbox.size_y / 2)

            label, score, colour = '', 0.0, _PALETTE[0]
            if det.results:
                hyp = det.results[0].hypothesis
                label = hyp.class_id
                score = hyp.score
                try:
                    colour = _PALETTE[int(label) % len(_PALETTE)]
                except (ValueError, TypeError):
                    colour = _PALETTE[0]

            cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
            text = f'{label} {score:.2f}'
            cv2.putText(img, text, (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)

        out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        out.header = img_msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
