#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class TurtleBotKinematics:
    def __init__(self):
        # Initialize the node
        rospy.init_node('turtlebot_kinematics', anonymous=True)

        # Subscriber to the /cmd_vel topic
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Publishers for wheel velocities
        self.left_wheel_pub = rospy.Publisher('/left_wheel_velocity', Float32, queue_size=10)
        self.right_wheel_pub = rospy.Publisher('/right_wheel_velocity', Float32, queue_size=10)

        # TurtleBot3 specifications
        self.wheel_radius = 0.033  # Wheel radius in meters
        self.wheel_base = 0.16     # Distance between wheels in meters

        rospy.loginfo("TurtleBot Kinematics node started.")
        rospy.spin()

    def cmd_vel_callback(self, twist):
        # Extract linear and angular velocities from Twist message
        linear_velocity = twist.linear.x
        angular_velocity = twist.angular.z

        # Inverse kinematics calculations to determine wheel velocities
        left_wheel_velocity = (linear_velocity - (self.wheel_base / 2.0) * angular_velocity) / self.wheel_radius
        right_wheel_velocity = (linear_velocity + (self.wheel_base / 2.0) * angular_velocity) / self.wheel_radius

        # Publish the wheel velocities
        self.publish_wheel_velocities(left_wheel_velocity, right_wheel_velocity)

        # If the robot should stop, set velocities to zero
        if linear_velocity == 0 and angular_velocity == 0:
            self.stop_robot()

    def publish_wheel_velocities(self, left_wheel_velocity, right_wheel_velocity):
        # Publish the left and right wheel velocities
        self.left_wheel_pub.publish(left_wheel_velocity)
        self.right_wheel_pub.publish(right_wheel_velocity)
        rospy.loginfo(f"Left Wheel Velocity: {left_wheel_velocity:.2f}, Right Wheel Velocity: {right_wheel_velocity:.2f}")

    def stop_robot(self):
        # Publish zero velocities to stop the robot
        self.left_wheel_pub.publish(0.0)
        self.right_wheel_pub.publish(0.0)
        rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    try:
        # Initialize and run the kinematics node
        TurtleBotKinematics()
    except rospy.ROSInterruptException:
        pass
