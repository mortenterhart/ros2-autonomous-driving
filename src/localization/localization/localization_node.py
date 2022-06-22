import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image, CompressedImage


class Localization(Node):
    def __init__(self):
        super().__init__('localization')

        # Subscribe to image and bbox topic
        self.subscriber_img_ = self.create_subscription(CompressedImage, '/proc_img', self.received_img, 10)
        self.subscriber_bboxes_ = self.create_subscription(Float32MultiArray, '/bounding_boxes', self.received_bbox, 10)

        self.img_width = 640
        self.img_height = 480
        self.ref_bbox_height_rel = 0.245833

        bbox_2m = [0.775391, 0.586111, 0.074219, 0.133333]
        bbox_corner_px = self.convert_rel_to_px_bbox(self.transform_center_to_corner_bbox_format(bbox_2m))
        print(f"Cone 2m angled distance: {self.compute_cone_distance(bbox_corner_px)}")

    def compute_cone_angle(self, bbox):
        bbox_center_bottom = (bbox[0] - 0.5, bbox[1] - bbox[3]/2)
        return np.arctan(bbox_center_bottom[0] / bbox_center_bottom[1])

    def compute_cone_distance(self, bbox):
        ref_bbox_height_px = self.img_height * self.ref_bbox_height_rel
        return ref_bbox_height_px / np.abs(bbox[1] - bbox[3]) * 100

    def transform_center_to_corner_bbox_format(self, center_bbox):
        return [center_bbox[0] - center_bbox[2]/2, center_bbox[1] - center_bbox[3]/2,
                center_bbox[0] + center_bbox[2]/2, center_bbox[1] + center_bbox[3]/2]

    def convert_rel_to_px_bbox(self, rel_bbox):
        return [rel_bbox[0] * self.img_width, rel_bbox[1] * self.img_height,
                rel_bbox[2] * self.img_width, rel_bbox[3] * self.img_height]

    def received_img(self, msg):
        print("Image:")
        print(msg.header)

    def received_bbox(self, msg):
        print("BBOX:")
        print(msg.header)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Localization())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
