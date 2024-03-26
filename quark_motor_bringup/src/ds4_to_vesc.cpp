#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ds4_driver_msgs/msg/status.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

const std::string ds4_status_topic = "/ds4/status";
const std::string duty_cycle_topic = "/vesc/commands/motor/duty_cycle";
const std::string servo_position_topic = "/vesc/commands/servo/position";

class MinimalNode : public rclcpp::Node
{
public:
  MinimalNode()
  : Node("minimal_node"), count_(0)
  {
    ds4_subscription_ = this->create_subscription<ds4_driver_msgs::msg::Status>(
      ds4_status_topic, 3, std::bind(&MinimalNode::subs_callback, this, _1));

    duty_cycle_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      duty_cycle_topic, 3);

    servo_position_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      servo_position_topic, 3);

    this->declare_parameter<bool>("disable_servo_control", false);
    this->get_parameter("disable_servo_control", this->disable_servo_control);


    RCLCPP_INFO_STREAM(
          this->get_logger(), "Node init sub topic: " << ds4_status_topic);

    RCLCPP_INFO_STREAM(
          this->get_logger(), "Duty cycle pub topic: " << duty_cycle_topic);

    RCLCPP_INFO_STREAM(
          this->get_logger(), "Servo pos pub topic: " << servo_position_topic);
  }

private:
  void subs_callback(const ds4_driver_msgs::msg::Status::SharedPtr msg)
  {
    auto duty_cycle_msg = std_msgs::msg::Float64();
    auto servo_position_msg = std_msgs::msg::Float64();

    duty_cycle_msg.data=msg->axis_right_y/2;

    servo_position_msg.data=msg->axis_left_x/2*0.7 + 0.5;

    duty_cycle_publisher_->publish(duty_cycle_msg);

    if(!this->disable_servo_control){
      servo_position_publisher_->publish(servo_position_msg);
    }

    RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          (*this->get_clock()), 5000, "Publishing transformed msgs");
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duty_cycle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_position_publisher_;

  rclcpp::Subscription<ds4_driver_msgs::msg::Status>::SharedPtr ds4_subscription_;

  size_t count_;
  bool disable_servo_control;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalNode>());
  rclcpp::shutdown();
  return 0;
}
