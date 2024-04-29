import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Twist

class PathPlanning(Node):
    def __init__ (self):
        super().__init("planning_algorithm")

def main(args = None):
    PathPlanning = rclpy.init(args = args)
    rclpy.spin(Node)
    
def path_planning() -> tuple:
    pass