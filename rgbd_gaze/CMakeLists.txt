cmake_minimum_required(VERSION 3.5)
project(rgbd_gaze)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_cmake
find_package(ament_cmake REQUIRED)
# ROS2 packages
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
# ROS2 interfaces
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eyelid_contour_msgs REQUIRED)
find_package(gaze_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Executables
set(EXECUTABLE_0 rgbd_gaze)
set(EXECUTABLE_1 calibration_eyeball)
set(EXECUTABLE_2 calibration_kappa)
set(EXECUTABLE_3 accuracy)

# Executable 0
add_executable(${EXECUTABLE_0} src/${EXECUTABLE_0}.cpp)
target_link_libraries(${EXECUTABLE_0} ${EIGEN3_LIBRARIES})
ament_target_dependencies(
  ${EXECUTABLE_0}
  rclcpp
  message_filters
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  gaze_msgs)

# Executable 1
add_executable(${EXECUTABLE_1} src/${EXECUTABLE_1}.cpp)
target_link_libraries(${EXECUTABLE_1} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(
  ${EXECUTABLE_1}
  rclcpp
  message_filters
  tf2
  pcl_conversions
  geometry_msgs
  sensor_msgs
  visualization_msgs
  eyelid_contour_msgs)

add_executable(${EXECUTABLE_2} src/${EXECUTABLE_2}.cpp)
target_link_libraries(${EXECUTABLE_2} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(
  ${EXECUTABLE_2}
  rclcpp
  message_filters
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  gaze_msgs)

add_executable(${EXECUTABLE_3} src/${EXECUTABLE_3}.cpp)
target_link_libraries(${EXECUTABLE_3} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(
  ${EXECUTABLE_3}
  rclcpp
  message_filters
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  gaze_msgs
  std_srvs)

# Install targets
install(TARGETS ${EXECUTABLE_0} ${EXECUTABLE_1} ${EXECUTABLE_2} ${EXECUTABLE_3}
        DESTINATION lib/${PROJECT_NAME})
# Install directories
install(DIRECTORY config launch users DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
