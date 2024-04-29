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

    def lidar_callback(self, msg):
        # Implement basic obstacle detection and avoidance
        # For simplicity, this example assumes obstacles directly in the path will trigger a stop
        if min(msg.ranges) < 0.5:  # Assume critical distance is 0.5 meters
            self.stop_moving()  # Stop if an obstacle is too close
        else:
            self.move_towards_target()  # Continue towards the target otherwise

    def odom_callback(self, msg):
        # Update robot's current position and orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def move_towards_target(self):
        # Simple proportional controller to steer towards the target
        twist = Twist()
        angle_to_target = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        angle_diff = self.normalize_angle(angle_to_target - self.current_yaw)
        
        if abs(angle_diff) > 0.1:  # If the heading error is significant
            twist.angular.z = 0.3 * angle_diff
        if math.hypot(self.target_x - self.current_x, self.target_y - self.current_y) > 0.1:
            twist.linear.x = 0.5  # Move forward only if aligned well

        self.velocity_publisher.publish(twist)

    def stop_moving(self):
        # Helper function to stop the robot
        twist = Twist()  # Zero velocity
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
