import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime

class ImgProcsessingNode(Node):
    def __init__(self):
        super().__init__('img_procsessing_node')
        self.publisher_ = self.create_publisher(Image, '/proc_img', 10)
        self.subscriber_img_ = self.create_subscription(Image, '/raw_image', self.callback, 10)

    def callback(self, msg):
        # forward the image data
        print(f"received new img {str(datetime.now()).split('.')[0]}")
        self.publisher_.publish(msg)

    def get_raw_img(self, msg):
        print(msg)

    def get_control_flow(self, msg):
        pass

    def send_proc_img(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImgProcsessingNode())

    rclpy.shutdown()
if __name__ == '__main__':
    main()
