#include <memory>
#include <string>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/header.hpp"
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

using namespace std;
using namespace cv;

static geometry_msgs::msg::PoseArray::SharedPtr detections_msg;

class ConePoseEstimator : public rclcpp::Node
{
public:
  ConePoseEstimator()
      : Node("cone_pose_estimator")
  {
    left_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_pose/left", 1); 
    front_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_pose/front", 1); 
    right_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_pose/right", 1); 
    back_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_pose/back", 1); 

    detections_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/camera/cone_centers", 1, std::bind(&ConePoseEstimator::detections_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Ready to receive images and detections!!");
  }


  void spin(){
    if(  detections_msg != nullptr
      // && front_img_msg != nullptr 
      // && right_img_msg != nullptr 
      // && left_img_msg != nullptr 
      // && back_img_msg != nullptr 
      ){
      estimate_poses();

      reset_msgs();
    }
  }

private:
  void reset_msgs(){
    detections_msg = nullptr;
  }

  void estimate_poses()
  {
    RCLCPP_INFO(this->get_logger(), "I heard: bananas");
    int h = 480;
    int w = 640;
    
    float f = 280;
    float cone_height = 0.07; //m
    float z_offset = 0.035;
    float gain = 3.157;

    geometry_msgs::msg::PoseArray left_detections;
    geometry_msgs::msg::PoseArray front_detections;
    geometry_msgs::msg::PoseArray right_detections;
    geometry_msgs::msg::PoseArray back_detections;

    for(geometry_msgs::msg::Pose pose: detections_msg->poses) {
      RCLCPP_INFO(this->get_logger(), "Pose");
      
      int img_num = pose.position.x / w;
      pose.position.x = (float)((int)pose.position.x % (int)w);

      //align for Rviz visualization
      pose.orientation.w = 0.707;
      pose.orientation.z = -0.707;

      pose.position.x -= w/2;
      pose.position.y -= h/2;

      float z = (f * (cone_height/2))/ (pose.position.y);
      pose.position.x = gain * (pose.position.x * z)/ (f);
      pose.position.y = 0; //always on ground
      pose.position.z = z * gain + z_offset;

      switch (img_num)
      {
      case 0:
        left_detections.poses.push_back(pose);
        break;

      case 1:
        front_detections.poses.push_back(pose);
        break;
      
      case 2:
        right_detections.poses.push_back(pose);
        break;
      
      case 3:
        back_detections.poses.push_back(pose);
        break;
      
      default:
        break;
      }
    }

    left_detections.header.stamp = rclcpp::Node::now(); // timestamp of creation of the msg
    left_detections.header.frame_id = "left_camera";
    left_det_pub->publish(left_detections);

    front_detections.header.stamp = rclcpp::Node::now(); // timestamp of creation of the msg
    front_detections.header.frame_id = "front_camera";
    front_det_pub->publish(front_detections);

    right_detections.header.stamp = rclcpp::Node::now(); // timestamp of creation of the msg
    right_detections.header.frame_id = "right_camera";
    right_det_pub->publish(right_detections);

    back_detections.header.stamp = rclcpp::Node::now(); // timestamp of creation of the msg
    back_detections.header.frame_id = "back_camera";
    back_det_pub->publish(back_detections);

    RCLCPP_INFO(this->get_logger(), "--------------------------------------");
  }

  void detections_callback(const geometry_msgs::msg::PoseArray::SharedPtr detections) const 
  {
    detections_msg = detections;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detections_sub;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr left_det_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr front_det_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr right_det_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr back_det_pub;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ConePoseEstimator> node = std::make_shared<ConePoseEstimator>();

  rclcpp::Rate rate(60);
  while(rclcpp::ok()){
    node->spin();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}