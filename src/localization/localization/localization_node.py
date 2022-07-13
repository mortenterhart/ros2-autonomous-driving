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
        # Publisher for known cones
        self.publisher_cones = self.create_publisher(Float32MultiArray, '/known_cones', 10)

        self.start_pos = None
        self.pos = np.array([0, 0], dtype=float)
        self.last_pos = np.array([0, 0], dtype=float)
        self.img_width = 640
        self.img_height = 480
        self.ref_bbox_height_rel = 0.245833
        self.scan_buffer = []
        self.angle_buffer = []
        self.FOV = 62

        bbox_2m = [0.775391, 0.586111, 0.074219, 0.133333]
        bbox_corner_px = self.convert_rel_to_px_bbox(self.transform_center_to_corner_bbox_format(bbox_2m))

        plt.ion()
        plt.show()
        plt.xlim(-2, 2)
        plt.ylim(0, 3)
        plt.legend()



    def received_scan(self, scan):
        # print("received scan")
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
            self.start_pos = np.copy(self.pos)


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
        pass

    def received_bbox(self, msg):
        bboxes = np.array(msg.data)
        bboxes = bboxes.reshape((-1, 6))

        if len(self.scan_buffer) < 5:
            return

        # get fov of buffer
        buffer_fov = [np.concatenate((item[int(-self.FOV/2):], item[:int(self.FOV/2)])) for item in self.scan_buffer]

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
            # print(f"custer {idx} - var: {cluster_var} - var_sum: {np.sum(cluster_var)}")


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
        # print(centroids.shape)
        if len(centroids) != 0:
            plt.scatter(centroids[:,0], centroids[:,1], alpha=.1)


        # sensor fusion
        bb_angles = []
        # TODO doesn't find the correct angles, check why this is happening...
        # centroid_distance = np.linalg.norm(centroids - (self.pos - self.start_pos), ord=2)
        # x_dist_centroid_points = centroids[:, 0] - (self.pos[0] - self.start_pos[0])
        # centroid_angles = np.degrees(np.arcsin(x_dist_centroid_points / centroid_distance)) + 31

        # print(f"centroid_angles: {centroid_angles}")
        print(f"cluster_angles: {cluster_angles}")

        fov_px_ratio = self.FOV / self.img_width

        centroid_classes = []  # classified centroids (by sensor fusion)
        # sort centroids by distance to the bot. use the sorted indices.
        if(len(centroids) == 0):
            return
        sorted_centroid_indices = np.argsort(np.linalg.norm(centroids - self.pos, ord=2, axis=1))
        # bool map whether a centroid is assigned to a bounding box
        used_centroids = np.zeros((len(sorted_centroid_indices)))

        # print(f"centroid: {centroids}")
        # print(f"centroid indices={sorted_centroid_indices}")
        # print(f"sorted bbs: {sorted(bboxes, key=lambda bb: bb[3] - bb[1], reverse=True)}")

        # print(f"bboxes: {len(bboxes)}, {len(sorted(bboxes, key=lambda bb: bb[3] - bb[1]))}")
        for bb in sorted(bboxes, key=lambda bb: bb[3] - bb[1], reverse=True):
            start_angle = bb[0] * fov_px_ratio
            end_angle = bb[2] * fov_px_ratio
            bb_angles.append((start_angle, end_angle))

            for idx in sorted_centroid_indices:
                if start_angle - 1 <= cluster_angles[idx] <= end_angle + 1 and not used_centroids[idx]:
                    centroid_classes.append((cluster_labels[idx], bb[5], centroids[idx]))
                    used_centroids[idx] = 1
                    break


        print(f"bb_angles: {bb_angles}")
        print(f"used_centroids: {used_centroids}")
        # print(f"centroid_classes: {centroid_classes}")

        detected_cones = np.empty((len(centroid_classes), 3))
        # print(f"{detected_cones.shape=}")
        for i, (_, label, centroid) in enumerate(centroid_classes):
            detected_cones[i] = label, centroid[0], centroid[1]

        cones_msg = Float32MultiArray()

        cones_msg.layout.dim.append(MultiArrayDimension())
        cones_msg.layout.dim.append(MultiArrayDimension())
        cones_msg.layout.dim[0].label = 'Detected Cones'
        cones_msg.layout.dim[0].size = len(detected_cones)
        cones_msg.layout.dim[1].label = 'label,x,y'
        cones_msg.layout.dim[1].size = 3

        cones_msg.data = detected_cones.flatten().tolist()

        self.publisher_cones.publish(cones_msg)
        # update cone map
        plt.draw()
        plt.pause(0.001)
        # plt.show()

    def plot_cluster(self, data, label, alpha):
        # data = points of a single cluster
        if len(data) == 0:
            return

        cluster_i = data
        cluster_var = np.var(cluster_i, axis=0)
        print(f"{label} {cluster_var} {cluster_i.shape[0]}")
        # if np.sum(cluster_var)<0.0005:
        if len(cluster_i) != 0:
            plt.scatter(cluster_i[:, 0], cluster_i[:, 1], s=5, alpha=alpha, label=f"{label}") #, c=cluster_labels



def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Localization())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
