import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import math
import argparse
import tf_transformations

class Navigator(Node):
    def __init__(self, target_x, target_y):
        super().__init__('navigator')
        self.target_x = target_x
        self.target_y = target_y
        
        self.lidar_subscription = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        self.obstacle_avoidance_active = False
        self.get_logger().info('Navigator started')

    def lidar_callback(self, msg):
        # Detect if there is an obstacle in front
        front_range = min(min(msg.ranges[0:15]), min(msg.ranges[-15:]))  # Check the front few degrees
        self.get_logger().info(f'Front range: {front_range}')
        if front_range < 1.5:  # Threshold for obstacle detection
            self.obstacle_avoidance_active = True
        else:
            self.obstacle_avoidance_active = False

        if self.obstacle_avoidance_active:
            self.get_logger().info('Avoiding obstacle')
            self.avoid_obstacle(msg)
        else:
            self.move_towards_target()

    def avoid_obstacle(self, msg):
        closest_range = min(msg.ranges)
        closest_angle = msg.ranges.index(closest_range) * msg.angle_increment
        twist = Twist()
        twist.angular.z = -0.5 if closest_angle > 0 else 0.5
        self.velocity_publisher.publish(twist)
        self.get_logger().info(f'Turning {"right" if closest_angle > 0 else "left"} to avoid obstacle')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.get_logger().info(f'Current position: ({self.current_x}, {self.current_y}), Yaw: {self.current_yaw}')

    def move_towards_target(self):
        twist = Twist()
        angle_to_target = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        angle_diff = self.normalize_angle(angle_to_target - self.current_yaw)
        twist.angular.z = 0.3 * angle_diff
        twist.linear.x = float(0.5 if abs(angle_diff) < 0.1 else 0.0)
        self.velocity_publisher.publish(twist)
        self.get_logger().info('Moving towards target')

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('target_x', type=float, help='Target X coordinate in meters')
    parser.add_argument('target_y', type=float, help='Target Y coordinate in meters')
    args = parser.parse_args()

    navigator = Navigator(args.target_x, args.target_y)
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
