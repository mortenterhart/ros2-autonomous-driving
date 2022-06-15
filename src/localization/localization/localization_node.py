import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image, CompressedImage

class Localization(Node):
    def __init__(self):
        super().__init__('localization')

        # Subscribe to image and bbox topic
        self.subscriber_img_ = self.create_subscription(CompressedImage, '/proc_img', self.received_img,10)
        self.subscriber_bboxes_ = self.create_subscription(Float32MultiArray, '/bounding_boxes', self.received_bbox, 10)

    def received_img(self, msg):
        print("Image:")
        print(msg.header)

    def received_bbox(self, msg):
        print("BBOX:")
        print(msg.header)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Localization())

    cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
