import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

class VisualizeVFH(Node):
    def __init__(self):
        super().__init__('VisualizeVFH')
        self.subscribe_histogram = self.create_subscription(Float32MultiArray, "histogram", self.histogram_callback, 10)

    def histogram_callback(self, msg):
        histogram = msg.data
        angles = range(len(histogram))  # Assuming each bin represents an equal angle slice
        plt.figure('Histogram')
        plt.bar(angles, histogram, width=1.0, color='b')
        plt.xlabel('Angle (bins)')
        plt.ylabel('Obstacle Proximity')
        plt.title('Polar Histogram')
        plt.draw()
        plt.pause(0.001)


def main(args = None):
    rclpy.init()
    node = VisualizeVFH()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()