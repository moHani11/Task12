#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class KinematicController:
    def __init__(self):
        rospy.init_node('kinematic_controller', anonymous=True)
        
        # Publishers to the left and right wheel velocities
        self.left_front_wheel_pub = rospy.Publisher('/left_front_wheel_velocity_controller/command', Float64, queue_size=10)
        self.right_front_wheel_pub = rospy.Publisher('/right_front_wheel_velocity_controller/command', Float64, queue_size=10)
        self.left_back_wheel_pub = rospy.Publisher('/left_back_wheel_velocity_controller/command', Float64, queue_size=10)
        self.right_back_wheel_pub = rospy.Publisher('/right_back_wheel_velocity_controller/command', Float64, queue_size=10)

        # Subscribe to the velocity commands (cmd_vel)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Robot parameters
        self.wheel_radius = 0.04
        self.wheel_base = 0.3     # Distance between the wheels

    def cmd_vel_callback(self, cmd_msg):
        # Extract linear and angular velocity from the command
        linear_velocity = cmd_msg.linear.x
        angular_velocity = cmd_msg.angular.z

        # Compute the wheel velocities using inverse kinematics
        right_wheel_velocity, left_wheel_velocity = self.inverse_kinematics(linear_velocity, angular_velocity)

        # Publish the velocities to each wheel
        self.left_front_wheel_pub.publish(left_wheel_velocity)
        self.right_front_wheel_pub.publish(right_wheel_velocity)
        self.left_back_wheel_pub.publish(left_wheel_velocity)
        self.right_back_wheel_pub.publish(right_wheel_velocity)

    def inverse_kinematics(self, linear_velocity, angular_velocity):
        # Compute right and left wheel velocities
        v_right = (2 * linear_velocity + angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        v_left = (2 * linear_velocity - angular_velocity * self.wheel_base) / (2 * self.wheel_radius)

        return v_right, v_left

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        kinematic_controller = KinematicController()
        kinematic_controller.run()
    except rospy.ROSInterruptException:
        pass
