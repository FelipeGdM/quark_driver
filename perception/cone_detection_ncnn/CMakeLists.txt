cmake_minimum_required(VERSION 3.8)
project(cone_detection_ncnn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
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

add_executable(camera_sub src/camera_sub.cpp)
ament_target_dependencies(camera_sub rclcpp sensor_msgs OpenCV cv_bridge image_transport)

install(TARGETS
  camera_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
