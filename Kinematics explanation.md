# TurtleBot Kinematics Node


## Description

The `turtlebot_kinematics.py` script subscribes to `/cmd_vel` to receive Twist messages from the PID controller (`pid_controller.py`). The script calculates the required wheel velocities using inverse kinematics and publishes them to control the TurtleBot's wheels. It also stops the robot once the goal position is reached by handling zero velocity commands.

## Node Details

- **Node Name:** `turtlebot_kinematics`
- **Subscribed Topic:** `/cmd_vel` (Type: `geometry_msgs/Twist`)
- **Published Topics:**
  - `/left_wheel_velocity` (Type: `std_msgs/Float32`)
  - `/right_wheel_velocity` (Type: `std_msgs/Float32`)

## Parameters

- **`wheel_radius`**: The radius of the TurtleBot's wheels in meters (default: `0.033 m`).
- **`wheel_base`**: The distance between the two wheels of the TurtleBot in meters (default: `0.16 m`).

## Code Explanation 

 1. Initialization

The node initializes by subscribing to the `/cmd_vel` topic to receive Twist messages that contain linear and angular velocity commands. It also sets up publishers for the left and right wheel velocities.

```python
rospy.init_node('turtlebot_kinematics', anonymous=True)
self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
self.left_wheel_pub = rospy.Publisher('/left_wheel_velocity', Float32, queue_size=10)
self.right_wheel_pub = rospy.Publisher('/right_wheel_velocity', Float32, queue_size=10)
```


2. Handling Velocity Commands

The cmd_vel_callback function is triggered when a message is received on the /cmd_vel topic. This function performs inverse kinematics calculations to determine the velocities of the left and right wheels.

```python
def cmd_vel_callback(self, twist):
    linear_velocity = twist.linear.x
    angular_velocity = twist.angular.z

    # Inverse kinematics calculations
    left_wheel_velocity = (linear_velocity - (self.wheel_base / 2.0) * angular_velocity) / self.wheel_radius
    right_wheel_velocity = (linear_velocity + (self.wheel_base / 2.0) * angular_velocity) / self.wheel_radius

    # Publish the wheel velocities
    self.publish_wheel_velocities(left_wheel_velocity, right_wheel_velocity)
```

3. Publishing Wheel Velocities
The calculated wheel velocities are published to the appropriate topics.

```python

def publish_wheel_velocities(self, left_wheel_velocity, right_wheel_velocity):
    self.left_wheel_pub.publish(left_wheel_velocity)
    self.right_wheel_pub.publish(right_wheel_velocity)
    rospy.loginfo(f"Left Wheel Velocity: {left_wheel_velocity:.2f}, Right Wheel Velocity: {right_wheel_velocity:.2f}")
```

4. Stopping the Robot

If the received velocities indicate a stop command, the robot stops by setting both wheel velocities to zero.

```python

def stop_robot(self):
    self.left_wheel_pub.publish(0.0)
    self.right_wheel_pub.publish(0.0)
    rospy.loginfo("Robot stopped.")
```


#### The node includes exception handling to safely shut down if a rospy.ROSInterruptException occurs.

python
```
if __name__ == '__main__':
    try:
        TurtleBotKinematics()
    except rospy.ROSInterruptException:
        pass
```


### Running the Node

1. Save the script as turtlebot_kinematics.py.
2.Make it executable:
```bash```
chmod +x turtlebot_kinematics.py
3.Run the script:
    ```bash``` 
     rosrun <your_package_name> turtlebot_kinematics.py


### note:

Ensure the PID controller (`pid_controller.py`) is running and publishing to `/cmd_vel`
