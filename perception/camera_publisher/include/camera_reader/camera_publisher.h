#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#ifndef CAMERA_PUBLISHER_H
#define CAMERA_PUBLISHER_H

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
  public:
    CameraPublisher();

  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

#endif //CAMERA_PUBLISHER_H
