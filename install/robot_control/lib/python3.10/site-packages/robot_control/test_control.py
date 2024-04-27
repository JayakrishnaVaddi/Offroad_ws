import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import argparse

class RobotController(Node):
    def __init__(self, target_x, target_y):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_x = target_x
        self.target_y = target_y
        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        # Here you would implement the logic to move towards the target coordinates
        # This example just prints the target for simplicity
        self.get_logger().info(f"Moving towards: {self.target_x}, {self.target_y}")

def main(args = None):
    parser = argparse.ArgumentParser(description='Navigate robot to a specific location.')
    parser.add_argument('--x', type=float, required=True, help='Target x coordinate')
    parser.add_argument('--y', type=float, required=True, help='Target y coordinate')
    args, unknown = parser.parse_known_args()  # Use parse_known_args to allow ROS 2 args

    rclpy.init(args=None)
    robot_controller = RobotController(args.x, args.y)
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
