# PID Controller for TurtleBot3

This script is a Python-based PID controller for the TurtleBot3, written for the ROS (Robot Operating System) environment. The controller uses PID (Proportional-Integral-Derivative) feedback loops to control both the linear and angular velocities of the TurtleBot3, allowing it to move towards a specific goal position and adjust its orientation.


## Script Breakdown

### 1. **Initialization and ROS Node Setup**

The script begins by importing necessary Python libraries and ROS message types:

- **`rospy`**: ROS Python client library.
- **`geometry_msgs.msg.Twist`**: ROS message type for velocity commands.
- **`nav_msgs.msg.Odometry`**: ROS message type for odometry data.

The `PIDController` class is then defined:

- **Node Initialization**: The ROS node is initialized using `rospy.init_node('pid_controller')`.
- **Publisher and Subscriber**:
  - `self.velocity_publisher` publishes velocity commands (`Twist` messages) to the TurtleBot3's velocity topic (`/cmd_vel`).
  - `self.odom_subscriber` subscribes to the TurtleBot3's odometry topic (`/odom`) to receive current position and orientation data.

### 2. **PID Constants and State Variables**

- **PID Constants**: Define the PID controller's parameters for linear and angular control:
  - `kp_linear`, `ki_linear`, `kd_linear`: Constants for the linear PID controller.
  - `kp_angular`, `ki_angular`, `kd_angular`: Constants for the angular PID controller.
- **State Variables**:
  - `self.current_x`, `self.current_y`, `self.current_theta`: Store the current position and orientation of the TurtleBot3.
  - `self.error_sum_linear`, `self.last_error_linear`: Variables to store the sum of linear errors and the last linear error for the PID calculation.
  - `self.error_sum_angular`, `self.last_error_angular`: Variables to store the sum of angular errors and the last angular error for the PID calculation.
  - `self.rate`: Sets the ROS loop rate to 10 Hz.

### 3. **Odometry Callback Function**

The `odom_callback` function updates the current position and orientation of the TurtleBot3:

- The function reads the current `x` and `y` coordinates from the odometry message (`msg`).
- The orientation is converted from quaternion format to Euler angles using trigonometric formulas, and then converted to degrees.

### 4. **Orientation Adjustment Function**

The `adjust_orientation` method adjusts the TurtleBot3's orientation to match a desired goal yaw (in degrees):

- It calculates the angular error between the current and goal yaw, normalizing it to the range `[-180, 180]` degrees.
- A PID controller is used to calculate the angular velocity needed to minimize the angular error.
- The velocity command is published to the TurtleBot3, and the orientation is adjusted until the error is within a 1-degree tolerance.

### 5. **Move to Goal Function**

The `move_to_goal` method controls the TurtleBot3's movement towards a specified target position (`goal_x`, `goal_y`):

- It calculates the distance to the goal and the desired orientation using trigonometric functions.
- PID controllers are used to compute both linear and angular velocities:
  - **Linear Control**: Based on the distance to the goal.
  - **Angular Control**: Based on the angular error between the current orientation and the desired orientation.
- The calculated velocities are published to the TurtleBot3, and the movement continues until the distance to the goal is within a 5 cm tolerance.

### 6. **Main Function Execution**

The `__main__` block takes user input for the goal coordinates and orientation:

- **User Input**: The user provides the target position (`goal_x`, `goal_y`) and desired yaw angle (`goal_yaw_degrees`).
- **Controller Initialization**: An instance of the `PIDController` class is created.
- **Function Execution Order**:
  - `move_to_goal` is executed first to move the TurtleBot3 to the target position.
  - `adjust_orientation` is executed next to adjust the TurtleBot3's orientation to match the desired yaw angle.
  
> **Important Note**: The `move_to_goal` function must be called before `adjust_orientation` to avoid errors.

### 7. **ROS Exception Handling**

The `rospy.ROSInterruptException` is used to handle any interruptions in the ROS node execution.

## Usage

To use this script, run it in a ROS environment with a TurtleBot3:

```bash
rosrun pid_control pid_turtlebot.py

