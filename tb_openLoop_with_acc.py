#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import csv  # Import the csv module

class OpenLoopControllerWithAcceleration(Node):
    def __init__(self):
        super().__init__('tb_openLoop_with_acc')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1  # Time count
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.speed = Twist()
        self.start_time = self.get_clock().now().to_msg().sec

        # Parameters for the scenario
        self.max_speed = 2.0  # Maximum speed after acceleration (in m/s)
        self.acceleration_distance = 5.0  # Acceleration Distance (m)
        self.total_distance = 10.0  # Total distance moved (m)

        # Using v^2 = u^2 + 2*a*s (u = 0, s = 5, v = 2 m/s)
        self.acceleration = (self.max_speed ** 2) / (2 * self.acceleration_distance)

        # Time to reach the max speed during acceleration
        self.acceleration_time = self.max_speed / self.acceleration

        # Remaining distance to cover at constant speed
        self.constant_distance = self.total_distance - self.acceleration_distance

        # Calculate time needed to cover the constant distance at max speed
        self.constant_time = self.constant_distance / self.max_speed

        # Total time i.e acceleration + constant speed phases
        self.total_time = self.acceleration_time + self.constant_time

        # Variables to track time, distance, and speed for plotting
        self.elapsed_time = 0.0
        self.current_distance = 0.0
        self.time_data = []
        self.distance_data = []
        self.speed_data = []

        self.get_logger().info(f"Scenario: Accelerate over {self.acceleration_distance} m to {self.max_speed} m/s, then travel {self.constant_distance} m at constant speed.")

    def timer_callback(self):
        # Get current time in seconds since the start
        current_time = self.get_clock().now().to_msg().sec - self.start_time
        self.elapsed_time += self.timer_period

        if current_time < self.acceleration_time:
            # Phase 1: Acceleration (using v = u + at)
            self.speed.linear.x = self.acceleration * current_time
            self.get_logger().info(f'Accelerating: {self.speed.linear.x:.2f} m/s')

        elif self.acceleration_time <= current_time < self.total_time:
            # Phase 2: Constant Speed (max_speed)
            self.speed.linear.x = self.max_speed
            self.get_logger().info(f'Moving at constant speed: {self.speed.linear.x:.2f} m/s')

        else:
            # Stop the robot once the desired point is reached
            self.speed.linear.x = 0.0
            self.publisher_.publish(self.speed)
            self.get_logger().info('Reached the desired point. Stopping...')
            
            # Plot the graphs for distance-time and speed-time
            self.plot_graphs()
            
            self.destroy_node()
            return

        # Calculate distance covered in the current time step
        self.current_distance += self.speed.linear.x * self.timer_period

        # Log data for graph
        self.time_data.append(self.elapsed_time)
        self.distance_data.append(self.current_distance)
        self.speed_data.append(self.speed.linear.x)

        # Publish the calculated speed
        self.publisher_.publish(self.speed)

    def plot_graphs(self):
        # Create a figure with two subplots: Distance vs Time and Speed vs Time
        plt.figure(figsize=(12, 5))

        # Subplot 1: Distance vs Time
        plt.subplot(1, 2, 1)
        plt.plot(self.time_data, self.distance_data, marker='o', color='b')
        plt.title('Distance vs Time Graph')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.grid()

        # Subplot 2: Speed vs Time
        plt.subplot(1, 2, 2)
        plt.plot(self.time_data, self.speed_data, marker='o', color='r')
        plt.title('Speed vs Time Graph')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.grid()

        # Show the plots
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    open_loop_acceleration = OpenLoopControllerWithAcceleration()
    rclpy.spin(open_loop_acceleration)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


