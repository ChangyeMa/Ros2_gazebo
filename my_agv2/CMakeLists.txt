cmake_minimum_required(VERSION 3.8)
project(my_agv2)


# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(example_position examples/example_position.cpp)
ament_target_dependencies(example_position
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(example_velocity examples/example_velocity.cpp)
ament_target_dependencies(example_velocity
  rclcpp
  std_msgs
)

add_executable(agv_lift examples/agv_lift.cpp)
ament_target_dependencies(agv_lift
  rclcpp
  std_msgs
)

add_executable(agv_unload examples/agv_unload.cpp)
ament_target_dependencies(agv_unload
  rclcpp
  std_msgs
)

install(
  DIRECTORY
    launch
    config
    urdf
    examples
    world
    rviz
  DESTINATION 
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()
## Install
install(
  TARGETS
    example_position
    example_velocity
    agv_lift
    agv_unload
  DESTINATION
    lib/${PROJECT_NAME}
)
ament_package()
