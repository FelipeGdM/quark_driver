cmake_minimum_required(VERSION 3.8)
project(planning)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(goal_planner src/goal_planner.cpp)
ament_target_dependencies(goal_planner rclcpp tf2 tf2_ros tf2_geometry_msgs sensor_msgs message_filters OpenCV geometry_msgs)

install(TARGETS
  goal_planner
  DESTINATION lib/${PROJECT_NAME})

add_executable(carrot_planner src/carrot_planner.cpp)
ament_target_dependencies(carrot_planner rclcpp tf2 tf2_ros tf2_geometry_msgs sensor_msgs message_filters OpenCV geometry_msgs)

install(TARGETS
  carrot_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  # launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()