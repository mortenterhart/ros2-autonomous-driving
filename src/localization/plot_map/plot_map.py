import rclpy

class Plot_map(Node):
    def __init__(self):
        super().__init__('Plot_map')

        self.subscriber_cones_ = sef.create_subscription(Float32MultiArray, '/known_cones', sef.receive_cones, 10)

    def receive_cones(self, msg):
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Plot_map())

    rclpy.shutdown()


if __name__ == '__main__':
    main()