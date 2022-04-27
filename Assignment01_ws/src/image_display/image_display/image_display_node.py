import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import os
from datetime import datetime

class ImgDisplayNode(Node):
    def __init__(self):
        super().__init__('img_display_node')
        # subscribe to the topic of processed images
        self.subscriber_img_ = self.create_subscription(Image, '/proc_img', self.display_img_data, 10)

        # set parameters and detect a parameter change
        self.store_imgs = False
        self.show_stream = True
        self.declare_parameter("store_imgs", value=self.store_imgs)
        self.declare_parameter("show_stream", value=self.show_stream)
        self.add_on_set_parameters_callback(self.on_param_change)

    def display_img_data(self, msg):
        # display the image data
        bridge = CvBridge()
        img_msg = bridge.imgmsg_to_cv2(msg)

        if self.show_stream:
            cv2.imshow("display", img_msg)
            cv2.waitKey(1)

        if self.store_imgs:
            path = "../imgs"
            if not os.path.exists(path):
                os.makedirs(path)

            cv2.imwrite(os.path.join(path, str(datetime.now()).replace(' ', '_')+'.jpg'), img_msg)

        print(f"{img_msg.shape}: {str(datetime.now()).split('.')[0]}")

    def on_param_change(self, parameters):
        for parameter in parameters:

            if parameter.name == "store_imgs":
                store_imgs = parameter.value
                print(f"changed store_imgs from {self.store_imgs} to {store_imgs}")
                self.store_imgs = store_imgs
                return SetParametersResult(successful=True)

            elif parameter.name == "show_stream":
                show_stream = parameter.value
                print(f"changed store_imgs from {self.show_stream} to {show_stream}")
                self.show_stream = show_stream
                cv2.destroyAllWindows()
                return SetParametersResult(successful=True)

            else:
                print(f"unknown/unused parameter {parameter.name}")
                return SetParametersResult(successful=False)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImgDisplayNode())

    cv2.destroyAllWindows()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
