cmake_minimum_required(VERSION 3.8)
project(static_cameras_tf2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf2_publisher src/cameras_tf2_publisher.cpp)
ament_target_dependencies(tf2_publisher rclcpp tf2 tf2_ros)

install(TARGETS
  tf2_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
