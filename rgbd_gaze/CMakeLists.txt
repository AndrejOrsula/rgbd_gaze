cmake_minimum_required(VERSION 3.5)
project(rgbd_gaze)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


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


## Dependencies
# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)

# ROS2 interfaces
find_package(rgbd_gaze_msgs REQUIRED)
find_package(openface_msgs REQUIRED)


## Install dirs
install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}
)


## Executables
set(EXECUTABLE_0 rgbd_gaze)
add_executable(${EXECUTABLE_0} src/${EXECUTABLE_0}.cpp)

# Executable 0
target_link_libraries(${EXECUTABLE_0}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(${EXECUTABLE_0}
  rclcpp
  message_filters
  rgbd_gaze_msgs
  openface_msgs
)
install(TARGETS
  ${EXECUTABLE_0}
  DESTINATION lib/${PROJECT_NAME}/
)


ament_package()
