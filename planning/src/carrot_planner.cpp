#include <memory>
#include <string>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <math.h> 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

using namespace std;
using namespace cv;

static geometry_msgs::msg::PoseArray::SharedPtr cone_positions;

class CarrotPlanner : public rclcpp::Node
{
public:
  CarrotPlanner()
      : Node("carrot_planner"), buffer_(this->get_clock()), listener_(buffer_)
  {
    cone_poses_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/cone_poses/base_link/all", 1, std::bind(&CarrotPlanner::cone_poses_callback, this, _1));
    goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1); 

    RCLCPP_INFO(this->get_logger(), "Ready to determine the goal!!");

    goal.header.frame_id = "odom";

    margin = 0.2;
  }

private:
  void cone_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr cones) 
  {
    if(std::size(cones->poses) >= 2) {
      geometry_msgs::msg::PoseStamped base_goal;
      base_goal.header.frame_id = "base_link";

      base_goal.pose.position.x = 0;
      base_goal.pose.position.y = 0;
      base_goal.pose.position.z = 0;

      // Get the 2 nearest cones
      sort(cones->poses.begin(), cones->poses.end(), [](const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) -> bool
      { 
        return sqrt(pow(a.position.x, 2) + pow(a.position.y, 2)) < sqrt(pow(b.position.x, 2) + pow(b.position.y, 2));
      });

      geometry_msgs::msg::Pose cone1 = cones->poses[0];
      geometry_msgs::msg::Pose cone2 = cones->poses[1];

      base_goal.pose.position.x = (cone1.position.x + cone2.position.x)/2;
      base_goal.pose.position.y = (cone1.position.y + cone2.position.y)/2;

      goal = buffer_.transform(base_goal, "odom");

      RCLCPP_INFO(this->get_logger(), "Goal set up!!");
    }
    goal.header.stamp = rclcpp::Node::now();
    goal_pub->publish(goal);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_poses_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
  geometry_msgs::msg::PoseStamped goal;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  float margin;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<CarrotPlanner> node = std::make_shared<CarrotPlanner>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}