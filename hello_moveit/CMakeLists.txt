cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(move_effector src/move_effector.cpp)
add_executable(move_joint src/move_joint.cpp)

target_include_directories(move_effector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(move_joint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(move_effector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(move_joint PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


ament_target_dependencies(
  move_effector 
  "moveit_ros_planning_interface"
  "rclcpp"
)
ament_target_dependencies(
  move_joint
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS move_effector 
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS move_joint
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
