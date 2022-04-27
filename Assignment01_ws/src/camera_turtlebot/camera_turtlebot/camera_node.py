import rclpy
import os
import cv2
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime

class Camera(Node):

    def __init__(self):
        super().__init__('camera')
        # publisher for raw img data
        self.publisher_ = self.create_publisher(Image, '/raw_image', 10)
        # camera stream
        self.vid = cv2.VideoCapture(0)

        # ros parameters
        self.freq = .2  # freq between images to publish [s]
        self.declare_parameter("fps", int(1 / self.freq))
        self.add_on_set_parameters_callback(self.on_param_change)
        # callback timer
        self.timer = self.create_timer(self.freq, self.callback)

    def callback(self):
        ret, frame = self.vid.read()
        bridge = CvBridge()

        try:
            msg = bridge.cv2_to_imgmsg(frame)
            self.publisher_.publish(msg)
            print(f"send new img {str(datetime.now()).split('.')[0]}")

        except CvBridgeError as e:
            print(e)

    def on_param_change(self, parameters):
        print("--------------------- PARAM CHANGE ---------------------")
        for parameter in parameters:
            if parameter.name == "fps":
                # calc new frequency
                fps = parameter.value
                print(f"changed fps from {1 / self.freq} to {fps}")
                self.freq = 1 / fps
                # reinitialize timer
                tmp = self.timer
                self.timer = self.create_timer(self.freq, self.callback)
                tmp.destroy()
                print("recreated timer")
                return SetParametersResult(successful=True)

        return SetParametersResult(successful=False)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Camera())

    cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
