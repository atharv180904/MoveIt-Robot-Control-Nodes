# ROS2_Moveit2_PandaArm

Here’s a `README.md` for both the `move_joint` and `move_effector` nodes:

---

# MoveIt Robot Control Nodes

This repository contains two ROS 2 nodes that demonstrate basic motion planning using MoveIt for controlling a robotic arm. The nodes are:

- `move_joint.cpp`: Moves the robot to specified joint angles.
- `move_effector.cpp`: Moves the robot's end effector to a specified pose.

Both nodes are designed to work with the `panda_arm` of a robot. They are examples of how to use the MoveIt API for joint space and Cartesian space planning.

## Prerequisites

Before running the nodes, ensure that you have the following installed:

- **ROS 2** (Galactic)
- **MoveIt2** for ROS 2
- A robot description (URDF or SRDF) for a robot arm that includes the `panda_arm` group. You can use the Panda robot (Franka Emika) model provided by MoveIt for simulation.

## Build Instructions

1. **Clone the repository**:
   ```bash
   git clone <repository_url> ~/your_ros2_workspace/src/
   ```

2. **Navigate to your ROS 2 workspace**:
   ```bash
   cd ~/your_ros2_workspace
   ```

3. **Build the package**:
   ```bash
   colcon build
   ```

4. **Source your workspace**:
   ```bash
   source install/setup.bash
   ```

## Running the Nodes

### 1. Joint Space Control (`move_joint.cpp`)

This node moves the robot's joints to specific angles (in radians, but specified in degrees in the code).

**Command to run**:
```bash
ros2 run <package_name> move_joint
```

### How It Works:
- It initializes the robot using MoveIt's `MoveGroupInterface`.
- It sets specific joint angles in degrees for the `panda_arm` and converts them to radians.
- It plans and executes the motion to achieve the specified joint positions.

### Code Highlights:
- The joint angles are defined in degrees:
  ```cpp
  std::vector<double> joint_angles_degrees = {89.0+10.0, -9.0, 0.0, -46.0, -3.0, 128.0, 29.0};
  ```
- The angles are converted to radians and used to plan the motion.

### 2. Cartesian Space Control (`move_effector.cpp`)

This node moves the robot’s end-effector (tool/hand) to a specific pose in the Cartesian space (position and orientation).

**Command to run**:
```bash
ros2 run <package_name> move_effector
```

### How It Works:
- It initializes the `panda_arm` using MoveIt's `MoveGroupInterface`.
- It sets a target pose (position and orientation in space) for the end-effector.
- It plans and executes the motion to achieve the target pose.

### Code Highlights:
- The pose is specified as follows:
  ```cpp
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;

  msg.position.x = 0.5;
  msg.position.y = 0.0;
  msg.position.z = 0.2;
  ```
- The node then uses MoveIt's planning API to plan and execute the motion.

## Dependencies

Both nodes rely on the following ROS 2 and MoveIt packages:

- `rclcpp`: For ROS 2 nodes and logging.
- `moveit_core`: Core MoveIt functionality.
- `moveit_ros_planning_interface`: For MoveIt planning and execution interfaces.
- `geometry_msgs`: For specifying end-effector target poses.

Make sure to have a valid MoveIt configuration for your robot (like the `panda_arm`) loaded into your ROS environment before running these nodes.

## Important Notes

1. **Robot Configuration**: Ensure that you have the appropriate MoveIt configuration for your robot. These examples assume the use of the Panda robot, but can be adapted to other robot configurations by modifying the group name from `"panda_arm"` to your robot’s arm planning group.

2. **Simulation vs Real Robot**: These examples are suitable for both simulation (e.g., in Gazebo) and real robots. Make sure you configure your ROS 2 environment accordingly.

3. **Motion Safety**: If you're using a real robot, ensure that you have proper safety measures in place, such as collision detection and velocity/acceleration limits.

## Troubleshooting

- **Planning Failures**: If the node fails to plan, ensure that your robot's environment and configuration are correct, and that there are no obstacles in the planning path.
- **Execution Failures**: If the motion execution fails, double-check that the planned trajectory is valid and that the robot's controllers are properly configured.

## Future Improvements

- Add error handling and retry mechanisms for planning and execution failures.
- Provide dynamic inputs for joint angles and end-effector poses instead of hardcoding values.
- Expand support to more robot models by making the planning group configurable via ROS parameters.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

This README file should provide clear instructions on how to build, run, and understand the two nodes. Let me know if you'd like to add or change anything!
