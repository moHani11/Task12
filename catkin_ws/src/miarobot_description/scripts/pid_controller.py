#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PIDController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('pid_controller', anonymous=True)
        
        # Publisher to the TurtleBot3 velocity topic
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to the TurtleBot3 odometry topic
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # PID constants
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.5
        
        self.kp_angular = 4.0
        self.ki_angular = 0.0
        self.kd_angular = 0.2

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # In degrees
        self.error_sum_linear = 0.0
        self.last_error_linear = 0.0
        self.error_sum_angular = 0.0
        self.last_error_angular = 0.0
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, msg):
        # Get current position and orientation from the odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert orientation from quaternion to Euler angles
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        theta_radians = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert radians to degrees
        self.current_theta = math.degrees(theta_radians)

    def adjust_orientation(self, goal_yaw_degrees):
        while not rospy.is_shutdown():
            # Calculate angular error in degrees
            angle_error_degrees = goal_yaw_degrees - self.current_theta

            # Normalize angle error to range [-180, 180] degrees
            angle_error_degrees = (angle_error_degrees + 180) % 360 - 180

            # Convert error to radians for PID calculations
            angle_error = math.radians(angle_error_degrees)

            # PID control for angular adjustment
            error_angular = angle_error
            self.error_sum_angular += error_angular
            delta_error_angular = error_angular - self.last_error_angular
            control_angular = (self.kp_angular * error_angular +
                               self.ki_angular * self.error_sum_angular +
                               self.kd_angular * delta_error_angular)
            self.last_error_angular = error_angular

            # Create and publish the velocity command for angular adjustment
            velocity_msg = Twist()
            velocity_msg.angular.z = max(min(control_angular, 0.2), -0.2)  # Limit angular speed

            self.velocity_publisher.publish(velocity_msg)

            # Check if the orientation is adjusted
            if abs(angle_error_degrees) < 1:  # 2.9 degrees tolerance (~0.05 rad)
                rospy.loginfo("Orientation adjusted! Stopping angular movement.")
                rospy.loginfo(f"Final position: yaw = {self.current_theta:.2f} degrees")

                stop_msg = Twist()
                self.velocity_publisher.publish(stop_msg)
                break

            self.rate.sleep()

    def move_to_goal(self, goal_x, goal_y):
        while not rospy.is_shutdown():
            # Compute distance to the goal
            distance = math.sqrt((goal_x - self.current_x)**2 + (goal_y - self.current_y)**2)
            desired_angle_radians = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
            desired_angle_degrees = math.degrees(desired_angle_radians)
            angle_error_degrees = desired_angle_degrees - self.current_theta

            # Normalize angle error to range [-180, 180] degrees
            angle_error_degrees = (angle_error_degrees + 180) % 360 - 180

            # Convert error to radians for PID calculations
            angle_error = math.radians(angle_error_degrees)

            # Linear PID calculations
            error_linear = distance
            self.error_sum_linear += error_linear
            delta_error_linear = error_linear - self.last_error_linear
            control_linear = (self.kp_linear * error_linear +
                              self.ki_linear * self.error_sum_linear +
                              self.kd_linear * delta_error_linear)
            self.last_error_linear = error_linear

            # Angular PID calculations
            error_angular = angle_error
            self.error_sum_angular += error_angular
            delta_error_angular = error_angular - self.last_error_angular
            control_angular = (self.kp_angular * error_angular +
                               self.ki_angular * self.error_sum_angular +
                               self.kd_angular * delta_error_angular)
            self.last_error_angular = error_angular

            # Create and publish the velocity command
            velocity_msg = Twist()
            velocity_msg.linear.x = min(control_linear, 0.2)  # Limit linear speed
            velocity_msg.angular.z = min(control_angular, 0.2)  # Limit angular speed

            self.velocity_publisher.publish(velocity_msg)

            # Check if the goal is reached
            if distance < 0.05:  # 5 cm tolerance
                rospy.loginfo("Goal reached! Stopping the TurtleBot3.")
                # Stop the TurtleBot3 by setting the velocities to zero
                stop_msg = Twist()
                self.velocity_publisher.publish(stop_msg)
                
                # Print the final position and orientation in degrees
                rospy.loginfo(f"Final position: x = {self.current_x:.2f}, y = {self.current_y:.2f}, yaw = {self.current_theta:.2f} degrees")
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Get target (x, y, yaw) from the user
        goal_x = float(input("Enter goal x coordinate: "))
        goal_y = float(input("Enter goal y coordinate: "))
        goal_yaw_degrees = float(input("Enter goal yaw angle (in degrees): "))

        # Initialize the PID controller
        controller = PIDController()
        
        # Move to the user-defined goal
        controller.move_to_goal(goal_x, goal_y)
        
        # Adjust orientation after reaching the goal
        controller.adjust_orientation(goal_yaw_degrees)

    except rospy.ROSInterruptException:
        pass