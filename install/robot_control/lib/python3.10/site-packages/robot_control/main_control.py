import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update)  # Adjust the timer as needed

    def update(self):
        msg = Twist()
        # Set linear and angular values
        msg.linear.x = 0.5  # Forward speed (m/s)
        msg.angular.z = 0.0  # Turn rate (rad/s)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
