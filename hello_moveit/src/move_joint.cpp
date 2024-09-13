#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <cmath>  // For M_PI

int main(int argc, char* argv[])
{
  
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit_joint_angles_degrees", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("hello_moveit_joint_angles_degrees");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  move_group_interface.setMaxVelocityScalingFactor(1.0);  // Maximum velocity
  move_group_interface.setMaxAccelerationScalingFactor(1.0);  // Maximum acceleration

  std::vector<double> joint_angles_degrees = {89.0+10.0, -9.0, 0.0, -46.0, -3.0, 128.0, 29.0};

  std::vector<double> joint_angles_radians ;
  joint_angles_radians.reserve(joint_angles_degrees.size());
  for (double angle_deg : joint_angles_degrees)
  {
    double angle_rad = angle_deg * M_PI / 180.0;  // Convert degrees to radians
    joint_angles_radians.push_back(angle_rad);
  }

  move_group_interface.setJointValueTarget(joint_angles_radians);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Execute the plan
  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful, executing...");
    auto execution_result = move_group_interface.execute(plan);
    if (execution_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Execution successful!");
    }
    else
    {
      RCLCPP_ERROR(logger, "Execution failed!");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
