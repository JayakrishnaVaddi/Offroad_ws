import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import math
import argparse
import tf_transformations
import numpy as np

from enum import Enum, auto

class State(Enum):
    NAVIGATING = auto()
    AVOIDING_OBSTACLE = auto()

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
        self.state = State.NAVIGATING
        self.avoidance_duration = 3.0 
        self.obstacle_clear_time = None

    def lidar_callback(self, msg):
        front_range = min(min(msg.ranges[0:30]), min(msg.ranges[-30:])) 
        if front_range < 2.5:  
                self.state = State.AVOIDING_OBSTACLE
                self.obstacle_clear_time = self.get_clock().now() 
        else:
            if self.state == State.AVOIDING_OBSTACLE:
                time_elapsed = (self.get_clock().now() - self.obstacle_clear_time).nanoseconds * 1e-9 
                if time_elapsed >= self.avoidance_duration:
                    self.state = State.NAVIGATING 
                
        if self.state == State.AVOIDING_OBSTACLE:
            self.avoid_obstacle(msg)
        else:
            self.move_towards_target()

    def avoid_obstacle(self, msg):
        front = min(min(msg.ranges[0:30]), min(msg.ranges[-30:]))
        left = min(msg.ranges[30:90])
        right = min(msg.ranges[-90:-30])

        twist = Twist()

        if front < 1.0:  
            if left > right:
                twist.angular.z = float(0.5)
            else:
                twist.angular.z = float(-0.5)
            
            twist.linear.x = float(0.1)
        else:
            twist.linear.x = float(0.5)  
            twist.angular.z = float(0)  

        self.velocity_publisher.publish(twist)


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def move_towards_target(self):
        twist = Twist()
        angle_to_target = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        angle_diff = self.normalize_angle(angle_to_target - self.current_yaw)
        
        twist.angular.z = 0.5 * angle_diff
        twist.linear.x = 0.5 if abs(angle_diff) < 0.1 else 0.0
        self.velocity_publisher.publish(twist)

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
