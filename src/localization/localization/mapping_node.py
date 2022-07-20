import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import matplotlib.pyplot as plt
import numpy as np


class FlexibleQueue:
    def __init__(self, max_length):
        self.queue = []

        self.size = 0
        self.max_length = max_length

    def push(self, x):
        self.queue.insert(0, x)

        if self.size < self.max_length:
            self.size += 1

        if self.size > self.max_length:
            self.queue.pop(self.max_length)

    def get(self, index):
        return self.queue[index]

    def __len__(self):
        return len(self.queue)


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')

        self.subscriber_potential_cones = self.create_subscription(Float32MultiArray, '/potential_cones', self.receive_cones, 10)
        self.subscriber_robot_pos = self.create_subscription(Float32MultiArray, '/robot_pos', self.receive_robot_pos, 10)
        self.publisher_known_cones = self.create_publisher(Float32MultiArray, '/known_cones', 10)

        self.BUFFER_LENGTH = 5
        self.TRACKING_RADIUS = 0.15

        self.cone_buffer = FlexibleQueue(self.BUFFER_LENGTH)
        self.known_cones = []

        self.robot_pos = []

        # plot settings
        plt.ion()
        plt.show()
        # plt.scatter(0, 0)
        # plt.xlim(-2, 2)
        # plt.ylim(0, 4)

    def receive_robot_pos(self, msg):
        self.robot_pos = np.array(msg.data)

    def receive_cones(self, msg):
        cones = np.array(msg.data).reshape((-1, msg.layout.dim[1].size))

        # add cones to buffer
        self.cone_buffer.push(cones)

        self.track_cones()
        self.plot_cones()
        self.publish_cones()

    def publish_cones(self):
        known_cones_msg = Float32MultiArray()

        known_cones_msg.layout.dim.append(MultiArrayDimension())
        known_cones_msg.layout.dim.append(MultiArrayDimension())
        known_cones_msg.layout.dim[0].label = 'Known Cones'
        known_cones_msg.layout.dim[0].size = len(self.known_cones)
        known_cones_msg.layout.dim[1].label = 'label,x,y'
        known_cones_msg.layout.dim[1].size = 3

        # Send known cones as flattened list
        known_cones_msg.data = np.array(self.known_cones).flatten().tolist()

        # Publish known cones
        self.publisher_known_cones.publish(known_cones_msg)

    def track_cones(self):
        if len(self.cone_buffer) < self.BUFFER_LENGTH:
            return

        # cone appearing in all buffers
        detected_cones = []

        for cone in self.cone_buffer.queue[0]:
            found_cones = [[cone[1], cone[2]]]

            for next_buffer in self.cone_buffer.queue[1:]:
                next_cone = self.find_cone_neighbor_in_list(cone, next_buffer)

                if next_cone is None:
                    break

                found_cones.append([next_cone[1], next_cone[2]])

            if len(found_cones) == self.BUFFER_LENGTH:
                mean_cone_pos = np.mean(found_cones, axis=0)
                detected_cones.append(np.array([cone[0], mean_cone_pos[0], mean_cone_pos[1]]))

        for detected_cone in detected_cones:
            known_cone = self.find_cone_neighbor_in_list(detected_cone, self.known_cones)

            if known_cone is None:
                self.known_cones.append(detected_cone)

    def find_cone_neighbor_in_list(self, cone, cone_buffer):
        for buffered_cone in cone_buffer:
            if self.compare_cones(cone, buffered_cone):
                return buffered_cone

        return None

    def compare_cones(self, cone_a, cone_b):
        # Return false if the labels do not match
        if int(cone_a[0]) != int(cone_b[0]):
            return False

        distance = np.linalg.norm(cone_a[1:] - cone_b[1:], ord=2)
        return distance < self.TRACKING_RADIUS

    def plot_cones(self):
        if len(self.known_cones) == 0:
            return

        # plot robot
        plt.scatter(self.robot_pos[0], self.robot_pos[1], c='red')

        # plot cones
        cone_colors = ['blue', 'orange', 'yellow']
        cones_np = np.array(self.known_cones)
        plt.scatter(cones_np[:, 1], cones_np[:, 2], c=[cone_colors[int(i)] for i in cones_np[:, 0]])

        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MappingNode())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
