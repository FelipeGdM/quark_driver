cmake_minimum_required(VERSION 3.8)
project(cone_detection_ncnn)

option(GPU_SUPPORT "Enable GPU support (Vulkan)" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS
  core highgui imgproc imgcodecs
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

list(APPEND CMAKE_PREFIX_PATH "/home/quark/Documents/ncnn/build/install/lib/cmake/ncnn/")
find_package(ncnn REQUIRED)
if (${ncnn_FOUND})
  message("-- NCNN found.")
  message("-- NCNN_VULKAN flag is ${NCNN_VULKAN}")
  if (${NCNN_VULKAN})
    message("-- AUTO-ENABLING GPU_SUPPORT")
    set(GPU_SUPPORT ON)
  endif()
  include_directories(${ncnn_INCLUDE})
endif()

if (GPU_SUPPORT)
  message("-- GPU support is ENABLED")
  find_package(Vulkan) # REQUIRES ncnn to be built with vulkan
  if (${VULKAN_FOUND})
    message("-- Vulkan found.")
  else()
    message("-- ERROR: AUTO-DISABLING GPU_SUPPORT, because Vulkan was not found")
    set(GPU_SUPPORT OFF)
  endif()
else()
  message("-- GPU support is DISABLED")
endif()

include_directories(
  include
  ${ncnn_INCLUDE}
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_sub src/camera_sub.cpp src/yolo.cpp)
ament_target_dependencies(camera_sub rclcpp sensor_msgs OpenCV cv_bridge image_transport)
target_link_libraries(camera_sub ${catkin_LIBRARIES} ncnn ${OpenCV_LIBS})

install(TARGETS
  camera_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
