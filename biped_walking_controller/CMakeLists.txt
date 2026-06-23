cmake_minimum_required(VERSION 3.8)
project(biped_walking_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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
# Add the executable and point it to the .cpp file
add_executable(biped_walking_node src/biped_walking_node.cpp)
target_include_directories(biped_walking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
add_executable(swing_trajectory_generator src/swing_trajectory_generator.cpp)
ament_target_dependencies(swing_trajectory_generator rclcpp geometry_msgs std_msgs)
# Link the ROS 2 dependencies and Eigen3 math library
ament_target_dependencies(biped_walking_node 
  rclcpp 
  geometry_msgs 
  Eigen3
  sensor_msgs
)

install(TARGETS
  biped_walking_node
  swing_trajectory_generator
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
