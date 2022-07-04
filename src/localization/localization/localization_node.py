import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from nav_msgs.msg import Odometry
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
        self.subscriber_odom = self.create_subscription(Odometry, 'odom', self.receive_odom, qos_profile=qos_profile_sensor_data)

        self.start_pos = None
        self.pos = np.array([0, 0], dtype=float)
        self.last_pos = np.array([0, 0], dtype=float)
        self.img_width = 640
        self.img_height = 480
        self.ref_bbox_height_rel = 0.245833
        self.scan_buffer = []
        self.points_received = 0

        bbox_2m = [0.775391, 0.586111, 0.074219, 0.133333]
        bbox_corner_px = self.convert_rel_to_px_bbox(self.transform_center_to_corner_bbox_format(bbox_2m))
        print(f"Cone 2m angled distance: {self.compute_cone_distance(bbox_corner_px)}")

    def received_scan(self, scan):
        if self.start_pos is None:
            return
        move_vec = self.pos - self.last_pos # movement of the turtlebot

        bot_movement = self.pos - self.start_pos

        print(f"bot_movement: {bot_movement}")
        print(f"pos: {self.pos}")

        self.last_pos = np.copy(self.pos)

        points_x = np.array(scan.ranges) * np.sin(np.linspace(0, 2 * np.pi, 360))
        points_y = np.array(scan.ranges) * np.cos(np.linspace(0, 2 * np.pi, 360))
        points = np.array([[x,y] for x, y in zip(points_x, points_y) if abs(x) >= 10e-10 and abs(y) >= 10e-10])
        points -= bot_movement
        self.scan_buffer.append(points)

        self.points_received += 1
        if self.points_received == 5:
            flat = np.concatenate(self.scan_buffer)
            cluster_labels = DBSCAN(eps=.1, min_samples=2).fit_predict(flat)

            num_clusters = len(np.unique(cluster_labels))

            for idx, label in enumerate(np.unique(cluster_labels)):

                cluster_i = flat[cluster_labels==label]
                # print(f"flat shape: {flat.shape}")

                # print(f"{np.unique(cluster_labels, return_counts=True)}")
                # print(flat.shape)
                cluster_var = np.var(cluster_i, axis=0)
                print(f"{label} {cluster_var} {cluster_i.shape[0]}")
                # if np.sum(cluster_var)<0.0005:
                plt.scatter(cluster_i[:, 0], cluster_i[:, 1], s=5, label=f"{label}") #, c=cluster_labels
            plt.legend()
            plt.show()

    def receive_odom(self, odom):
        self.pos[0] = odom.pose.pose.position.x
        self.pos[1] = odom.pose.pose.position.y

        if self.start_pos is None:
            self.start_pos = self.pos

        print(f"odom updated: {self.pos}")

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
