import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime

class ImgProcsessingNode(Node):
    def __init__(self):
        super().__init__('img_procsessing_node')
        self.publisher_ = self.create_publisher(CompressedImage, '/proc_img', 10)
        self.subscriber_img_ = self.create_subscription(Image, '/raw_image', self.callback, 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        # forward the image data
        print(f"received new img {str(datetime.now()).split('.')[0]}")

        img = self.bridge.imgmsg_to_cv2(msg)

        # Convert image to compressed image
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(img)        

        # Publish compressed image
        self.publisher_.publish(compressed_image)

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
