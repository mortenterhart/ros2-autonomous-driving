import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class ImgDisplayNode(Node):
    def __init__(self):
        super().__init__('img_display_node')
        # self.publisher_ = self.create_publisher(Image, '/proc_img', 10)
        self.subscriber_img_ = self.create_subscription(Image, '/proc_img', self.callback, 10)

    def callback(self, msg):
        # display the image data
        bridge = CvBridge()
        img_msg = bridge.imgmsg_to_cv2(msg)
        cv2.imshow("display", img_msg)
        print("Received data:")
        print(img_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImgDisplayNode())

    rclpy.shutdown()
if __name__ == '__main__':
    main()
