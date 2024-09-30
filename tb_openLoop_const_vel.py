#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import csv  # Import the csv module

class OpenLoopControllerConstantSpeed(Node):
    def __init__(self):
        super().__init__('tb_openLoop_const_vel')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 1.0  # Time count
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.speed = Twist()
        self.speed.linear.x = 0.5  # constant linear speed(m/s)

        self.distance = 10.0  # goal distance(m)
        self.speed = self.speed.linear.x  # Speed in m/s
        self.time_to_run = self.distance / self.speed  # Time to goal distance
        self.elapsed_time = 0.0  # Variable to track elapsed time

        # Variables to track time, distance, and speed for plotting
        self.time_data = []
        self.distance_data = []
        self.speed_data = []  # New variable to store speed data
        self.current_distance = 0.0

    def timer_callback(self):
        if self.elapsed_time < self.time_to_run:
            self.publisher_.publish(self.speed)  # Keep moving at constant speed
            self.get_logger().info(f'Moving at constant speed: {self.speed.linear.x:.2f} m/s')
            
            # Increment elapsed time and calculate distance
            self.elapsed_time += self.timer_period
            self.current_distance += self.speed.linear.x * self.timer_period
            
            # Append data for graph
            self.time_data.append(self.elapsed_time)
            self.distance_data.append(self.current_distance)
            self.speed_data.append(self.speed.linear.x)  # Store the constant speed value

        else:
            # Stop the robot after reaching the goal distance
            self.speed.linear.x = 0.0
            self.publisher_.publish(self.speed)
            self.get_logger().info('Goal distance reached. Stopping the robot.')
            self.timer.cancel()  # Stop the timer to cease publishing messages
            
            # Plot the graphs for distance-time and speed-time
            self.plot_graphs()

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

    def save_to_csv(self):
        # Save the time, distance, and speed data to a CSV file
        with open('distance_speed_time_data.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Distance (m)', 'Speed (m/s)'])  # Write header
            for t, d, v in zip(self.time_data, self.distance_data, self.speed_data):
                writer.writerow([t, d, v])
        self.get_logger().info('Data saved to distance_speed_time_data.csv.')

def main(args=None):
    rclpy.init(args=args)
    const_speed_controller = OpenLoopControllerConstantSpeed()
    rclpy.spin(const_speed_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


