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
        # self.subscriber_img_ = self.create_subscription(CompressedImage, '/proc_img', self.received_img, 10)
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
        self.angle_buffer = []
        self.points_received = 0

        bbox_2m = [0.775391, 0.586111, 0.074219, 0.133333]
        bbox_corner_px = self.convert_rel_to_px_bbox(self.transform_center_to_corner_bbox_format(bbox_2m))
        print(f"Cone 2m angled distance: {self.compute_cone_distance(bbox_corner_px)}")

    def received_scan(self, scan):
        if self.start_pos is None:
            return

        bot_movement = self.pos - self.start_pos

        # print(f"bot_movement: {bot_movement}")
        # print(f"pos: {self.pos}")

        # transform the lidar data to 2d points
        points_x = np.array(scan.ranges) * np.sin(np.flip(np.linspace(0, 2 * np.pi, 360)))
        points_y = np.array(scan.ranges) * np.cos(np.flip(np.linspace(0, 2 * np.pi, 360)))
        points = np.array([[x,y] for x, y in zip(points_x, points_y) if abs(x) >= 10e-10 and abs(y) >= 10e-10])

        # normalize with the movement of the bot (s.t. points are stationary)
        points += bot_movement

        if len(self.scan_buffer) > 5:
            self.scan_buffer.pop(0)
        self.scan_buffer.append(points)

    def receive_odom(self, odom):
        self.pos[0] = odom.pose.pose.position.x
        self.pos[1] = odom.pose.pose.position.y

        if self.start_pos is None:
            self.start_pos = self.pos

        # print(f"odom updated: {self.pos}")

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
        bboxes = np.array(msg.data)
        bboxes = bboxes.reshape((-1, 6))

        if len(self.scan_buffer) < 5:
            return

        # get fov of buffer
        buffer_fov = [np.concatenate((item[-31:], item[:31])) for item in self.scan_buffer]

        # find cluster
        flat_buffer = np.concatenate(buffer_fov)
        point_labels = DBSCAN(eps=.1, min_samples=2).fit_predict(flat_buffer)
        # self.plot_cluster(flat_buffer, point_labels)


        # cluster centroids
        clusters = []
        cluster_labels = []
        cluster_angles = []

        clusters_no_cone = []
        cluster_labels_no_cone = []

        for idx, label in enumerate(np.unique(point_labels)):
            cluster = flat_buffer[point_labels==label]
            cluster_indices = np.argwhere(point_labels == label)
            cluster_degrees = cluster_indices % 62
            cluster_degrees = np.ones(cluster_degrees.shape) * 62 - cluster_degrees
            cluster_var = np.var(cluster, axis=0)
            print(f"custer {idx} - var: {cluster_var} - var_sum: {np.sum(cluster_var)}")


            # filter by cluster variance
            if np.sum(cluster_var)<0.002 and label != -1:
                clusters.append(cluster)
                cluster_labels.append(label)
                cluster_angles.append(np.mean(cluster_degrees))
            else:
                clusters_no_cone.append(cluster)
                cluster_labels_no_cone.append(label)

        for cluster, label in zip(clusters, cluster_labels):
            self.plot_cluster(cluster, label, 1)
        for cluster, label in zip(clusters_no_cone, cluster_labels_no_cone):
            self.plot_cluster(cluster, label, .1)

        # calc cluster centroids
        centroids = [np.mean(cluster, axis=0) for cluster in clusters]
        centroids = np.array(centroids)
        plt.scatter(centroids[:,0], centroids[:,1], alpha=.1)

        plt.xlim(-2, 2)
        plt.ylim(0, 3)
        plt.legend()


        # sensor fusion
        bb_angles = []
        # TODO doesn't find the correct angles, check why this is happening...
        # centroid_distance = np.linalg.norm(centroids - (self.pos - self.start_pos), ord=2)
        # x_dist_centroid_points = centroids[:, 0] - (self.pos[0] - self.start_pos[0])
        # centroid_angles = np.degrees(np.arcsin(x_dist_centroid_points / centroid_distance)) + 31

        # print(f"centroid_angles: {centroid_angles}")
        print(f"cluster_angles: {cluster_angles}")

        fov_px_ratio = 62 / self.img_width

        centroid_classes = []  # classified centroids (by sensor fusion)
        # print(f"bboxes: {len(bboxes)}, {len(sorted(bboxes, key=lambda bb: bb[3] - bb[1]))}")
        for bb in sorted(bboxes, key=lambda bb: bb[3] - bb[1], reverse=True):
            start_angle = bb[0] * fov_px_ratio
            end_angle = bb[2] * fov_px_ratio
            bb_angles.append((start_angle, end_angle))

            for centroid, label, angle in zip(centroids, cluster_labels, cluster_angles):
                if start_angle -.5 <= angle <= end_angle + .5:
                    centroid_classes.append((label, bb[5]))
                    # TODO pop centroid afterwards.
            # make cluster- and bbox length equal
            # give centroids color of bbx
        print(f"bb_angles: {bb_angles}")
        print(f"centroid_classes: {centroid_classes}")
        # update cone map
        plt.show()

    def plot_cluster(self, data, label, alpha):
        # data = points of a single cluster

        cluster_i = data
        cluster_var = np.var(cluster_i, axis=0)
        print(f"{label} {cluster_var} {cluster_i.shape[0]}")
        # if np.sum(cluster_var)<0.0005:
        plt.scatter(cluster_i[:, 0], cluster_i[:, 1], s=5, alpha=alpha, label=f"{label}") #, c=cluster_labels



def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Localization())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
