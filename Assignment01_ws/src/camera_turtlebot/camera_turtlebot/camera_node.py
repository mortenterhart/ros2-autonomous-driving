import rclpy
import os
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# import picamera
# import picamera.array

class Camera(Node):

        def __init__(self):
                super().__init__('camera')
                self.publisher_ = self.create_publisher(Image, '/raw_image', 10)
                self.freq = 2
                self.timer = self.create_timer(self.freq, self.callback)
                self.vid = cv2.VideoCapture(0)

        def callback(self):
                #print(os.getcwd())

                # img_msg = Image(data =)
                # take image TODO:



                # publish image
                # empty_img = np.zeros((10,10,3))
                # empty_img[:, :3, 1] = 240
                # empty_img = empty_img / 255
                # print(empty_img.shape)
                ret, frame = self.vid.read()
                bridge = CvBridge()



                try:
                        msg = bridge.cv2_to_imgmsg(frame)
                        self.publisher_.publish(msg)
                        print(msg)

                except CvBridgeError as e:
                        print(e)

        def capture(self):
                with picamera.PiCamera() as camera:
                        with picamera.array.PiRGBArray(camera) as output:
                                camera.capture(output, 'rgb')
                                print('Captured %dx%d image' % (output.array.shape[1], output.array.shape[0]))
                                return output.array




def main(args=None):
        rclpy.init(args=args)
        rclpy.spin(Camera())
        rclpy.shutdown()

if __name__ == '__main__':
    main()
