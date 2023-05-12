#include <memory>
#include <string>
#include <cstring>
#include <iostream>
#include <stdio.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"
// #include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_msgs/msg/odometry.hpp"

#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

using namespace std;

static geometry_msgs::msg::PoseArray::SharedPtr cone_positions;

const float V_TRESHOLD = 5;

class MotorController : public rclcpp::Node
{
public:
  MotorController()
  : Node("goal_planner"), buffer_(this->get_clock()), listener_(buffer_)
  {

    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1,
      std::bind(&MotorController::goal_callback, this, _1)
    );

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1,
      std::bind(&MotorController::odom_callback, this, _1)
    );

    lin_pub = this->create_publisher<std_msgs::msg::Float64>(
      "/vesc/commands/motor/duty_cycle", 3
    );

    ang_pub = this->create_publisher<std_msgs::msg::Float64>(
      "/vesc/commands/servo/position", 3
    );

    this->declare_parameter<bool>("invert_angular", false);
    this->declare_parameter<double>("angular_proportional", 0.4);
    this->declare_parameter<double>("linear_proportional", 1);
    this->declare_parameter<double>("max_lin_vel", 0.2);

    this->get_parameter("angular_proportional", this->angular_proportional);
    this->get_parameter("linear_proportional", this->linear_proportional);
    this->get_parameter("invert_angular", this->invert_angular);
    this->get_parameter("max_lin_vel", this->max_lin_vel);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MotorController::spin, this));

    RCLCPP_INFO(this->get_logger(), "Ready to follow goal");
  }

private:

  // double _gamma(double beta, double psi_dot, double v_x, double v_y, double gamma_0){
  //   v = std::sqrt(v_x*v_x + v_y*v_y);
  //   if(v < V_TRESHOLD){
  //     return gamma_0/2;
  //   }else{ 
  //     return v*(A11/v*np.abs(beta) + A12/v**2*np.abs(psi_dot)) + gamma_0/2;
  //   }
  // }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
  {
    // goal = buffer_.transform(goal_msg, "base_link");
    goal_msg->header.stamp = rclcpp::Time(0);
    goal = buffer_.transform(*goal_msg, "base_link");
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_msg = *msg;
  }

  void spin()
  {

    static float distance, delta_x, delta_y, theta, angular_action, linear_action;
    static std_msgs::msg::Float64 ang_msg, lin_msg;
    
    distance = std::sqrt(delta_x*delta_x + delta_y*delta_y);
    delta_x = goal.pose.position.x;
    delta_y = goal.pose.position.y;

    theta = std::atan2(delta_y, delta_x);

    linear_action = std::min(std::abs(this->max_lin_vel), std::abs(this->linear_proportional*distance));
    
    if(delta_x < 0){
      linear_action *= -1;
      theta = std::atan2(delta_y, -delta_x);
    }

    angular_action = this->angular_proportional*theta;

    angular_action = angular_action/2 + 0.5;

    if(invert_angular){
      angular_action *= -1;
    }

    ang_msg.data = angular_action;
    lin_msg.data = linear_action;

    this->ang_pub->publish(ang_msg);

    if(this->max_lin_vel!=0.0){
      this->lin_pub->publish(lin_msg);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Spin(): Ang " << angular_action << " Lin " << linear_action);
    RCLCPP_INFO_STREAM(this->get_logger(), "Spin(): D x " << delta_x << " D y " << delta_y);

  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lin_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ang_pub;

  geometry_msgs::msg::PoseStamped goal;
  nav_msgs::msg::Odometry odom_msg;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  bool reached_first_point;

  double angular_proportional;
  double linear_proportional;
  double max_lin_vel;

  bool invert_angular;

rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<MotorController> node = std::make_shared<MotorController>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
