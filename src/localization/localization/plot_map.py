import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np


class Plot_map(Node):
    def __init__(self):
        super().__init__('Plot_map')

        self.subscriber_cones_ = self.create_subscription(Float32MultiArray, '/known_cones', self.receive_cones, 10)

        plt.ion()
        plt.show()
        plt.scatter(0, 0)
        # plt.xlim(-2, 2)
        # plt.ylim(0, 4)

    def receive_cones(self, msg):
        data = np.array(msg.data).reshape((-1, msg.layout.dim[1].size))
        plt.scatter(data[0, 1], data[0, 2], c='red')

        cone_colors = ['blue', 'orange', 'yellow']
        plt.scatter(data[1:, 1], data[1:, 2], c=[cone_colors[int(i)] for i in data[1:, 0]])
        plt.draw()
        plt.pause(0.001)
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Plot_map())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
