import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

class Localization(Node):
    def __init__(self):
        super().__init__('localization')

        # Subscribe to image and bbox topic
        self.subscriber_img_ = self.create_subscription(CompressedImage, '/proc_img', self.received_img, 10)
        self.subscriber_bboxes_ = self.create_subscription(Float32MultiArray, '/bounding_boxes', self.received_bbox, 10)
        qos = QoSProfile(depth=10)
        self.subscriber_scan = self.create_subscription(LaserScan, 'scan', self.received_scan, qos_profile=qos_profile_sensor_data)

        self.img_width = 640
        self.img_height = 480
        self.ref_bbox_height_rel = 0.245833
        self.scan_buffer = []
        self.points_received = 0

        bbox_2m = [0.775391, 0.586111, 0.074219, 0.133333]
        bbox_corner_px = self.convert_rel_to_px_bbox(self.transform_center_to_corner_bbox_format(bbox_2m))
        print(f"Cone 2m angled distance: {self.compute_cone_distance(bbox_corner_px)}")

    def received_scan(self, scan):
        points_x = np.array(scan.ranges) * np.sin(np.linspace(0, 2 * np.pi, 360))
        points_y = np.array(scan.ranges) * np.cos(np.linspace(0, 2 * np.pi, 360))
        points = np.array([[x,y] for x, y in zip(points_x, points_y) if abs(x) >= 10e-10 and abs(y) >= 10e-10])
        self.scan_buffer.append(points)

        self.points_received += 1
        if self.points_received == 5:
            flat = np.concatenate(self.scan_buffer)
            cluster_labels = DBSCAN(eps=.1, min_samples=2).fit_predict(flat)
            print(f"{np.unique(cluster_labels, return_counts=True)}")
            print(flat.shape)
            plt.scatter(flat[:, 0], flat[:, 1], s=5, c=cluster_labels)
            plt.show()
            print(points_x)

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
