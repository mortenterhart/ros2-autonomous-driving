import rclpy
import cv2
import os
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# from perception.models.common import DetectMultiBackend


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')
        # subscribe to the topic of processed images
        self.subscriber_img_ = self.create_subscription(CompressedImage, '/proc_img', self.detect_cones, 10)
        self.model = torch.load('./models/yolov5.pt')

    def detect_cones(self, msg):
        # display the image data
        bridge = CvBridge()
        img = bridge.compressed_imgmsg_to_cv2(msg)

        # Convert BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        detection = self.model(img)    

        bboxes = detection.xyxy[0]

        for bbox in bboxes:
            cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), thickness=2)

        cv2.imshow('Detected cones', img)
        cv2.waitKey(0)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ConeDetectionNode())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
