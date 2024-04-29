import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
import argparse

class ControlAlgorithm(Node):
    def __init__(self, target_x, target_y):
        super().__init__('control_algorithm')
        self.target_x = target_x
        self.target_y = target_y
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription

    def lidar_callback(self, msg):

        ranges = np.array(msg.ranges)
        min_distance = np.min(ranges[ranges > 0])
        self.get_logger().info(f'Closest obstacle at {min_distance} meters')

        twist = Twist()
        if min_distance < 1.0:
            twist.linear.x = 0.0  
            twist.angular.z = 0.5 
        else:
            twist.linear.x = 0.5
            twist.angular.z = 0.0  
        self.publisher.publish(twist)


def main(args = None):
    parser = argparse.ArgumentParser(description='Navigate robot to a specific location.')
    parser.add_argument('--x', type=float, required=True, help='Target x coordinate')
    parser.add_argument('--y', type=float, required=True, help='Target y coordinate')
    args, unknown = parser.parse_known_args()  # Use parse_known_args to allow ROS 2 args

    rclpy.init(args=None)
    robot_controller = ControlAlgorithm(args.x, args.y)
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
