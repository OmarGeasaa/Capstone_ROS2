cmake_minimum_required(VERSION 3.8)
project(biped_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(custom_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)        

# 2. INCLUDE DIRECTORIES
include_directories(include)

# 3. BUILD EXECUTABLES
# --- ANKLE NODES ---
add_executable(ankle_ik_node src/ankle_ik_node.cpp)
target_include_directories(ankle_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ankle_ik_node rclcpp std_msgs custom_interfaces)

add_executable(ankle_fk_node src/ankle_fk_node.cpp)
target_include_directories(ankle_fk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ankle_fk_node rclcpp sensor_msgs custom_interfaces)

add_executable(upper_body_controller src/upper_body_controller.cpp)
target_include_directories(upper_body_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(upper_body_controller rclcpp sensor_msgs custom_interfaces std_msgs)


# --- HIP NODES ---
add_executable(hip_ik_node src/hip_ik_node.cpp)
target_include_directories(hip_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hip_ik_node rclcpp geometry_msgs custom_interfaces)
target_link_libraries(hip_ik_node Eigen3::Eigen)


add_executable(hip_fk_node src/hip_fk_node.cpp)
target_include_directories(hip_fk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hip_fk_node rclcpp sensor_msgs custom_interfaces)
target_link_libraries(hip_fk_node Eigen3::Eigen)

# 4. INSTALLATION
# Install the compiled nodes
install(TARGETS
  ankle_ik_node
  ankle_fk_node
  hip_ik_node
  hip_fk_node
  upper_body_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the headers for clean architecture
install(DIRECTORY include/
  DESTINATION include
)

# 5. TESTING
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()