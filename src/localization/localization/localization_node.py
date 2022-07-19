import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import math


class NpQueue():
    def __init__(self, maxQLen, elemDim):
        self.q = np.zeros((maxQLen, elemDim))

        self.currSize = 0
        self.maxQLen = maxQLen
        self.elemDim = elemDim

    def push(self, x):
        self.q[1:] = self.q[:-1]
        self.q[0] = x

        if self.currSize < self.maxQLen:
            self.currSize += 1

def header_to_float_stamp(header):
    return float(f"{header.stamp.sec}.{header.stamp.nanosec}")


class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        qos = QoSProfile(depth=10)

        # Subscribe to image and bbox topic
        self.subscriber_bboxes_ = self.create_subscription(Float32MultiArray, '/bounding_boxes', self.received_bbox, 10)
        self.subscriber_scan = self.create_subscription(LaserScan, 'scan', self.received_scan, qos_profile=qos_profile_sensor_data)
        self.subscriber_odom = self.create_subscription(Odometry, 'odom', self.receive_odom, qos_profile=qos_profile_sensor_data)
        # Publisher for known cones
        self.publisher_cones = self.create_publisher(Float32MultiArray, '/known_cones', 10)

        self.start_pos = None
        self.pos = np.array([0, 0], dtype=float)
        self.start_orientation = None
        self.orientation = 0
        self.img_width = 640
        self.img_height = 480
        self.scan_buffer = NpQueue(30, 721)   # buffer for single
        self.odom_buffer = NpQueue(30, 4)
        self.angle_buffer = []
        self.FOV = 62
        self.VAR_THRESHOLD = 0.002

        plt.ion()
        plt.show()
        plt.xlim(-2, 2)
        plt.ylim(0, 3)
        plt.legend()


    def lidar_data_to_point_cloud(self, ranges):
        # ranges are indexed by angle, and describe the distance until the lidar hit an objrct.
        points_x = np.array(ranges) * np.sin(np.flip(np.linspace(0, 2 * np.pi, 360)))
        points_y = np.array(ranges) * np.cos(np.flip(np.linspace(0, 2 * np.pi, 360)))
        points = np.array([[x,y] for x, y in zip(points_x, points_y)])

        return points

    def transform_points(self, points, stamp):
        # get the correct odom data depending on timestamp
        odom_idx = np.argmin(np.abs(self.odom_buffer.q[:, -1] - stamp))
        odom = self.odom_buffer.q[odom_idx]

        pos = odom[:2]
        orientation = odom[2]

        # translation
        bot_movement = pos - self.start_pos # self.pos - self.start_pos
        points += bot_movement

        # rotation
        angle = orientation - self.start_orientation # self.orientation - self.start_orientation
        angle = -angle
        c, s = np.cos(angle), np.sin(angle)
        R = np.array(((c, -s), (s, c)))
        points = points @ R

        print(f"angle={angle * 180 / np.pi}")

        return points

    def received_scan(self, scan):
        if self.start_pos is None:
            return

        stamp = header_to_float_stamp(scan.header)
        # transform the lidar data to 2d points
        points = self.lidar_data_to_point_cloud(scan.ranges)

        # normalize with the movement of the bot (s.t. points are stationary)
        points = self.transform_points(points, stamp)

        points = self.add_time_stamp(points.flatten(), stamp)

        self.scan_buffer.push(points)

    def add_time_stamp(self, arr, stamp):
        return np.concatenate([arr, np.array([stamp])])

    def receive_odom(self, odom):
        self.pos[0] = odom.pose.pose.position.x
        self.pos[1] = odom.pose.pose.position.y

        self.orientation = euler_from_quaternion(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )[2]

        stamp = float(f"{odom.header.stamp.sec}.{odom.header.stamp.nanosec}")
        self.odom_buffer.push(np.array([self.pos[0], self.pos[1], self.orientation, stamp]))

        if self.start_pos is None:
            self.start_pos = np.copy(self.pos)
            self.start_orientation = self.orientation

    def received_bbox(self, msg):
        bboxes = np.array(msg.data)
        bboxes = bboxes.reshape((-1, msg.layout.dim[1].size))
        stamp_sec = int(bboxes[0, 0])
        stamp_nano = int(bboxes[0, 1])
        stamp = float(f"{stamp_sec}.{stamp_nano}")

        if self.scan_buffer.currSize < 5:
            return

        # get the synchronized scan slices
        scan_idx = np.argmin(np.abs(self.scan_buffer.q[:, -1] - stamp))
        scans = self.scan_buffer.q[scan_idx:scan_idx+5, :-1].reshape(-1, 360, 2)
        print(self.scan_buffer.q)
        print(f"{scan_idx}")
        print(f"{self.scan_buffer.q.shape=}")
        print(f"{self.scan_buffer.q[scan_idx-4:scan_idx+1, :-1]=}")
        print(scans.shape)

        # get fov of buffer
        buffer_fov = [np.concatenate((item[int(-self.FOV/2):], item[:int(self.FOV/2)])) for item in scans]

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
            cluster_degrees = cluster_indices % self.FOV
            cluster_degrees = np.ones(cluster_degrees.shape) * self.FOV - cluster_degrees
            cluster_var = np.var(cluster, axis=0)
            # print(f"custer {idx} - var: {cluster_var} - var_sum: {np.sum(cluster_var)}")


            # TODO remove the cluster at (0,0) -> the cluster of points where no object was hit
            # filter by cluster variance
            if np.sum(cluster_var)<self.VAR_THRESHOLD and label != -1:
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
        # print(f"used_centroids: {used_centroids}")
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

        # Send robot position with label -1 first, then the detected cones
        cones_msg.data = [-1.0, self.pos[0], self.pos[1]] + detected_cones.flatten().tolist()

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
        # print(f"{label} {cluster_var} {cluster_i.shape[0]}")
        # if np.sum(cluster_var)<0.0005:
        if len(cluster_i) != 0:
            plt.scatter(cluster_i[:, 0], cluster_i[:, 1], s=5, alpha=alpha, label=f"{label}") #, c=cluster_labels

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Localization())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
