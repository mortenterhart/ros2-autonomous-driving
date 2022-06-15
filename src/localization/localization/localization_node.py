import rclpy

class Localization(Node):
    super().__init__('localization')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Localization())

    cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
