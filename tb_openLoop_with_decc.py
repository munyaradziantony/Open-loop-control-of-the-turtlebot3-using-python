#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import csv  # Import the csv module

class OpenLoopControllerWithDeceleration(Node):
    def __init__(self):
        super().__init__('tb_openLoop_with_decc')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 1.0  # Time count
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.speed = Twist()
        self.start_time = self.get_clock().now().to_msg().sec

        # Scenario parameters
        self.acceleration_distance = 3.0  # acceleration distance (m)
        self.constant_speed_distance = 4.0  # constant speed distance (s)
        self.deceleration_distance = 3.0  # deceleration distance (s)
        self.total_distance = self.acceleration_distance + self.constant_speed_distance + self.deceleration_distance

        # Motion parameters
        self.acceleration = 0.5  # Acceleration(m/s^2)
        self.deceleration = 0.5  # Deceleration (m/s^2)
        self.current_distance = 0.0
        self.current_phase = 'acceleration'  # either: acceleration, constant_speed, deceleration

        # Variables to store distance, time, and speed data for plotting
        self.elapsed_time = 0.0
        self.time_data = []
        self.distance_data = []
        self.speed_data = []

    def timer_callback(self):
        # Calculate the time since the start of the program
        current_time = self.get_clock().now().to_msg().sec - self.start_time
        self.elapsed_time += self.timer_period

        # Acceleration stage
        if self.current_phase == 'acceleration':
            self.speed.linear.x += self.acceleration * self.timer_period  # v = u + at
            self.current_distance += self.speed.linear.x * self.timer_period
            self.get_logger().info(f'Accelerating: {self.speed.linear.x:.2f} m/s, Distance: {self.current_distance:.2f} m')

            if self.current_distance >= self.acceleration_distance:
                # Move to the next phase: Constant Speed
                self.current_phase = 'constant_speed'
                self.speed.linear.x = self.speed.linear.x  # Keep the current speed as the constant speed

        # Constant Speed stage
        elif self.current_phase == 'constant_speed':
            self.current_distance += self.speed.linear.x * self.timer_period
            self.get_logger().info(f'Constant Speed: {self.speed.linear.x:.2f} m/s, Distance: {self.current_distance:.2f} m')

            if self.current_distance >= self.acceleration_distance + self.constant_speed_distance:
                # Move to the next phase: Deceleration
                self.current_phase = 'deceleration'

        # Deceleration stage
        elif self.current_phase == 'deceleration':
            self.speed.linear.x -= self.deceleration * self.timer_period  # v = u - at
            if self.speed.linear.x < 0.0:
                self.speed.linear.x = 0.0
            self.current_distance += self.speed.linear.x * self.timer_period
            self.get_logger().info(f'Decelerating: {self.speed.linear.x:.2f} m/s, Distance: {self.current_distance:.2f} m')

            if self.current_distance >= self.total_distance or self.speed.linear.x <= 0.0:
                # Stop the robot once deceleration phase is complete
                self.speed.linear.x = 0.0
                self.publisher_.publish(self.speed)
                self.get_logger().info('Reached the goal position. Stopping...')
                
                # Plot the graphs for distance-time and speed-time
                self.plot_graphs()
                
                # Save data to a CSV file
                self.save_to_csv()
                
                self.destroy_node()
                return

        # Log data for graph and CSV file
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

    def save_to_csv(self):
        # Save the distance, time, and speed data to a CSV file
        with open('distance_speed_time_data.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Distance (m)', 'Speed (m/s)'])  # Write header
            for t, d, v in zip(self.time_data, self.distance_data, self.speed_data):
                writer.writerow([t, d, v])
        self.get_logger().info('Data saved to distance_speed_time_data.csv.')

def main(args=None):
    rclpy.init(args=args)
    open_loop_deceleration = OpenLoopControllerWithDeceleration()
    rclpy.spin(open_loop_deceleration)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
