cmake_minimum_required(VERSION 3.8)
project(aruco_camera_pose_estimator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(calibration_srv REQUIRED)

add_executable(aruco_camera_pose_estimator
  src/pose_service.cpp
  src/image_processor.cpp
)
ament_target_dependencies(aruco_camera_pose_estimator
  rclcpp
    sensor_msgs
    geometry_msgs
    cv_bridge
    calibration_srv
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include
)
target_link_libraries(aruco_camera_pose_estimator
  Eigen3::Eigen
  ${OpenCV_LIBS}
)
install(TARGETS
  aruco_camera_pose_estimator
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
