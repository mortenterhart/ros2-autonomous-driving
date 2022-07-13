import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np


class Plot_map(Node):
    def __init__(self):
        super().__init__('Plot_map')

        self.subscriber_cones_ = self.create_subscription(Float32MultiArray, '/known_cones', self.receive_cones, 10)

        self.cone_buffer = []
        self.BUFFER_LENGTH = 5
        self.TRACKING_THRESHOLD_X = 0.1
        self.TRACKING_THRESHOLD_Y = 0.1
        self.cones = []

        self.robot_pos = [0.0, 0.0]

        # plot settings
        plt.ion()
        plt.show()
        # plt.scatter(0, 0)
        # plt.xlim(-2, 2)
        # plt.ylim(0, 4)

    def receive_cones(self, msg):
        data = np.array(msg.data).reshape((-1, msg.layout.dim[1].size))

        # get robot pos
        if data[0, 0] == -1.0:
            self.robot_pos = [data[0, 1], data[0, 2]]

        # add cones to buffer
        if len(self.cone_buffer) > self.BUFFER_LENGTH:
            self.cone_buffer.pop(0)
        self.cone_buffer.append(data[1:])

        self.track_cones()
        self.plot_cones()

    def track_cones(self):
        if len(self.cone_buffer) < self.BUFFER_LENGTH:
            return

        # cone appearing in all buffers
        new_cones = []

        # detect cones that appear in all buffer-states
        for cone in self.cone_buffer[0]:
            cone_positions = [[cone[1],cone[2]]]
            no_cone_found = False

            # compare to other cone-set for matching pos
            for buffered_cone_set in self.cone_buffer[1:]:
                no_cone_found = True

                # compare to cones in given set
                for buffered_cone in buffered_cone_set:
                    # compare label/position
                    label_matching = int(cone[0]) == int(buffered_cone[0])
                    x_matching = cone[1] > buffered_cone[1] - self.TRACKING_THRESHOLD_X and cone[1] < buffered_cone[1] + self.TRACKING_THRESHOLD_X
                    y_matching = cone[2] > buffered_cone[2] - self.TRACKING_THRESHOLD_Y and cone[2] < buffered_cone[2] + self.TRACKING_THRESHOLD_Y
                    if x_matching and y_matching and label_matching:
                        # position matching
                        cone_positions.append([buffered_cone[1], buffered_cone[2]])
                        no_cone_found = False
                        break

                # no matching cone found in one set
                if no_cone_found:
                    break

            if no_cone_found:
                continue

            # cone in all buffers found
            cone_pos = np.mean(cone_positions, axis=0)
            new_cones.append(np.concatenate(cone[0], cone_pos))

        # compare to cones on the map
        for new_cone in new_cones:
            for old_cone in self.cones:
                # compare label/position
                label_matching = int(new_cone[0]) == int(old_cone[0])
                x_matching = old_cone[1] > new_cone[1] - self.TRACKING_THRESHOLD_X and old_cone[1] < new_cone[1] + self.TRACKING_THRESHOLD_X
                x_matching = old_cone[2] > new_cone[2] - self.TRACKING_THRESHOLD_Y and old_cone[2] < new_cone[2] + self.TRACKING_THRESHOLD_Y
                if label_matching and x_matching and y_matching:
                    new_cones.remove(new_cone)
                    break

        # add new cones to map
        self.cones.extend(new_cones)

    def plot_cones(self):
        # plot robot
        plt.scatter(self.robot_pos[0], self.robot_pos[1], c='red')

        # plot cones
        cone_colors = ['blue', 'orange', 'yellow']
        plt.scatter(self.cones[:, 1], self.cones[:, 2], c=[cone_colors[int(i)] for i in self.cones[1:, 0]])

        plt.draw()
        plt.pause(0.001)
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Plot_map())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
