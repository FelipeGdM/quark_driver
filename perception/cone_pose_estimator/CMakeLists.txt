cmake_minimum_required(VERSION 3.8)
project(cone_pose_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(stereo_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_estimator src/cone_pose_estimator.cpp)
ament_target_dependencies(pose_estimator rclcpp tf2 tf2_ros tf2_geometry_msgs sensor_msgs message_filters cv_bridge image_transport OpenCV geometry_msgs stereo_msgs)

install(TARGETS
  pose_estimator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
