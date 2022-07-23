import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Twist

class Navigation(Node):
    def __init__(self):
        super().__init__("navigation")

        self.subscriber_cones_ = self.create_subscription(Float32MultiArray, '/known_cones', self.received_cones, 10)
        self.subscriber_pos_ = self.create_subscription(Float32MultiArray, '/robot_pos', self.update_pos, 10)

        self.publish_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.cones = None # cones published by map
        self.old_pos = np.array([0, -1]) # old roboter position
        self.bot_pos = np.array([0, 0])  # current roporter position

        self.target = np.array([0, 1]) # position of target

        self.BLUE = 0
        self.ORANGE = 1
        self.YELLOW = 2


    def received_cones(self, msg):
        self.cones = np.array(msg.data).reshape((-1, msg.layout.dim[1].size))

        # get cones in front of bot
        v_movement = bot_pos - old_pos
        is_in_front = np.dot(cones[1:], v_movement) > 0
        
        cones_in_front = cones[is_in_front]

        # abort if no cones in front
        if cones_in_front.size == 0:
                target = None
                return

        # get closest yellow and blue
        cones_blue = cones_in_front[cones_in_front[:,0] == self.BLUE]
        cones_yellow =  cones_in_front[cones_in_front[:,0] == self.YELLOW]

        closest_blue_ind = np.argmin(np.linalg.norm(cones_blue[1:] - bot_pos, 2))
        closest_blue = cones_blue[closest_blue_ind]

        closet_yellow_ind = np.argmin(np.linalg.norm(cones_yellow[1:] - bot_pos, 2))
        closet_yellow = cones_yellow[closet_yellow_ind]

        # calculate target
        self.target = (closest_blue[1:] + closet_yellow[1:]) / 2



    def update_pos(self, msg):
        old_pos = bot_pos
        bot_pos = np.array(msg.data).reshape((-1, msg.layout.dim[1].size))

        move_to_point()

    def move_to_point():
        twist_msg = Twist()

        if target == None:
            twist_msg.linear[2] = 0
            twist_msg.angular[2] = 0

            publish_cmd_vel.publish(twist_msg)


            
        trajectory = target - bot_pos
        angle = np.arctan2(trajectory[0], trajectory[1]) - np.arctan2(bot_pos[0], bot_pos[1])


        twist_msg.linear[2] = 0.1
        twist_msg.angular[2] = angle/10

        publish_cmd_vel.publish(twist_msg)




def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Navigation())

    rclpy.shutdown()


if __name__ == '__main__':
    main()