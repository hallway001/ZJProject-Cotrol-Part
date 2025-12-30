cmake_minimum_required(VERSION 3.8)
project(loader_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ament_index_python REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Check if loader_control_interfaces exists, otherwise provide stub
find_package(loader_control_interfaces QUIET)
if(NOT loader_control_interfaces_FOUND)
  message(WARNING "loader_control_interfaces not found. Please create this package or install it.")
endif()

# Include directories
include_directories(include)

# Controller implementations
add_library(pid_controller SHARED
  src/controllers/pid_controller.cpp
)
ament_target_dependencies(pid_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)
target_include_directories(pid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(pure_pursuit_controller SHARED
  src/controllers/pure_pursuit_controller.cpp
)
ament_target_dependencies(pure_pursuit_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)
target_include_directories(pure_pursuit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(mpc_controller SHARED
  src/controllers/mpc_controller.cpp
)
ament_target_dependencies(mpc_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)
target_include_directories(mpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpc_controller Eigen3::Eigen)

add_library(smc_controller SHARED
  src/controllers/smc_controller.cpp
)
ament_target_dependencies(smc_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)
target_include_directories(smc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Utility libraries
add_library(command_modulator SHARED
  src/utils/command_modulator.cpp
)
ament_target_dependencies(command_modulator
  rclcpp
  loader_control_interfaces
)
target_include_directories(command_modulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(trajectory_interpolator SHARED
  src/utils/trajectory_interpolator.cpp
)
ament_target_dependencies(trajectory_interpolator
  nav_msgs
)
target_include_directories(trajectory_interpolator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(safety_monitor SHARED
  src/utils/safety_monitor.cpp
)
ament_target_dependencies(safety_monitor
  loader_control_interfaces
  nav_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(safety_monitor
  trajectory_interpolator
)
target_include_directories(safety_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(ukf_estimator SHARED
  src/state_estimator/ukf_estimator.cpp
)
ament_target_dependencies(ukf_estimator
  loader_control_interfaces
)
target_include_directories(ukf_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Node executables
add_executable(trajectory_tracker_node src/nodes/trajectory_tracker_node.cpp)
ament_target_dependencies(trajectory_tracker_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  nav_msgs
  loader_control_interfaces
)
target_link_libraries(trajectory_tracker_node
  pid_controller
  pure_pursuit_controller
  mpc_controller
  smc_controller
  trajectory_interpolator
)
target_include_directories(trajectory_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(command_modulator_node src/nodes/command_modulator_node.cpp)
ament_target_dependencies(command_modulator_node
  rclcpp
  rclcpp_components
  loader_control_interfaces
)
target_link_libraries(command_modulator_node
  command_modulator
)
target_include_directories(command_modulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(state_estimator_node src/nodes/state_estimator_node.cpp)
ament_target_dependencies(state_estimator_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  loader_control_interfaces
)
target_link_libraries(state_estimator_node
  ukf_estimator
)
target_include_directories(state_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(control_manager_node src/nodes/control_manager_node.cpp)
ament_target_dependencies(control_manager_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_srvs
  lifecycle_msgs
  nav_msgs
  loader_control_interfaces
)
target_link_libraries(control_manager_node
  safety_monitor
  trajectory_interpolator
)
target_include_directories(control_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(TARGETS
  pid_controller
  pure_pursuit_controller
  mpc_controller
  smc_controller
  command_modulator
  trajectory_interpolator
  safety_monitor
  ukf_estimator
  trajectory_tracker_node
  command_modulator_node
  state_estimator_node
  control_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  test/test_control_performance.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

