import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
import matplotlib.animation as animation

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.positions = []  # List to store x, y coordinates

        # Set up the plot
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.line, = self.ax.plot([], [], 'bo-', markersize=4)  # Initialize empty line
        self.ax.set_title('Path of the Robot')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.grid(True)

        # Start the matplotlib GUI loop
        plt.ion()  # Turn on interactive mode
        plt.show()

    def odom_callback(self, msg):
        # Extract the position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions.append((x, y))  # Append the position tuple to the list

        # Immediately update the plot
        self.update_plot()

    def update_plot(self):
        if self.positions:
            x_vals, y_vals = zip(*self.positions)
            self.line.set_data(x_vals, y_vals)
            self.ax.relim()  # Recalculate limits
            self.ax.autoscale_view(True, True, True)  # Rescale the view
            self.fig.canvas.draw()  # Redraw the figure
            self.fig.canvas.flush_events()  # Process GUI events

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
