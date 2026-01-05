cmake_minimum_required(VERSION 3.8)
project(loader_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Include directories
include_directories(include)

# Source files
set(SOURCES
  src/data_receiver.cpp
  src/actuator_model.cpp
  src/controller_base.cpp
  src/pid_controller.cpp
  src/pure_pursuit_controller.cpp
  src/steering_error_monitor.cpp
  src/control_node.cpp
)

# Header files
set(HEADERS
  include/loader_control/data_receiver.hpp
  include/loader_control/actuator_model.hpp
  include/loader_control/controller_base.hpp
  include/loader_control/pid_controller.hpp
  include/loader_control/pure_pursuit_controller.hpp
  include/loader_control/steering_error_monitor.hpp
  include/loader_control/vehicle_state.hpp
)

# Create library
add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  message_filters
  diagnostic_msgs
  visualization_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Create executable
add_executable(${PROJECT_NAME}_node src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  message_filters
  diagnostic_msgs
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

# Set RPATH for executables to find the shared library
# In ROS 2, libraries are installed to lib/${PROJECT_NAME}, so we need to set RPATH
set_target_properties(${PROJECT_NAME}_node PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Export the library so it can be found by other packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  message_filters
  diagnostic_msgs
  visualization_msgs
)

# Create simulation node executable
add_executable(simulation_node src/simulation_node.cpp)
ament_target_dependencies(simulation_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  simulation_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(PROGRAMS
  scripts/param_merge.py
  scripts/error_analyzer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

