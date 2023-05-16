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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/header.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
      : Node("cone_pose_estimator"), buffer_(this->get_clock()), listener_(buffer_)
  {
    left_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/left", 1); 
    front_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/front", 1); 
    right_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/right", 1); 
    back_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/back", 1); 
    base_link_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses/base_link/all", 1); 

    detections_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/camera/cone_centers", 1, std::bind(&ConePoseEstimator::detections_callback, this, _1));

    dist_thresh = 0.05;

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

    geometry_msgs::msg::PoseArray all_detections;

    for(geometry_msgs::msg::Pose pose: detections_msg->poses) {
      RCLCPP_INFO(this->get_logger(), "Pose");
      
      int image_row = pose.position.y / h;
      int image_column = pose.position.x / w;
      pose.position.x = (float)((int)pose.position.x % (int)w);
      pose.position.y = (float)((int)pose.position.y % (int)h);

      //align for Rviz visualization
      pose.orientation.w = 0.707;
      pose.orientation.z = -0.707;

      pose.position.x -= w/2;
      pose.position.y -= h/2;

      float z = (f * (cone_height/2))/ (pose.position.y);
      pose.position.x = gain * (pose.position.x * z)/ (f);
      pose.position.y = 0.137; //always on ground
      pose.position.z = z * gain + z_offset;

      geometry_msgs::msg::PoseStamped pose_from_cam;
      pose_from_cam.header.stamp = rclcpp::Node::now();
      pose_from_cam.pose = pose;

      switch (image_row)
      {
      case 0:
        switch (image_column)
        {
          case 0:
            pose_from_cam.header.frame_id = "left_camera";
            left_detections.poses.push_back(pose);
            break;

          case 1:
            pose_from_cam.header.frame_id = "front_camera";
            front_detections.poses.push_back(pose);
            break;
        }
        break;

      case 1:
        switch (image_column)
        {
          case 0:
            pose_from_cam.header.frame_id = "right_camera";
            right_detections.poses.push_back(pose);
            break;
          
          case 1:
            pose_from_cam.header.frame_id = "back_camera";
            back_detections.poses.push_back(pose);
            break;
        }
        break;
      
      default:
        break;
      }

      geometry_msgs::msg::PoseStamped pose_from_base = buffer_.transform(pose_from_cam, "base_link");
      all_detections.poses.push_back(pose_from_base.pose);
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

    // bool found;
    // for(auto old_pose: previous_all_detections.poses){
    //   found = false;
    //   for(int i = 0; i < size(all_detections.poses); i++){
    //     auto new_pose = all_detections.poses[i];

    //     float dist = sqrt(pow(old_pose.position.x - new_pose.position.x, 2) + pow(old_pose.position.y - new_pose.position.y, 2));
        
    //     if(dist <= dist_thresh){
    //       all_detections.poses[i].position.x = (new_pose.position.x + old_pose.position.x)/2;
    //       all_detections.poses[i].position.y = (new_pose.position.y + old_pose.position.y)/2;
          
    //       found = true;
    //       break;
    //     }
    //   }

    //   if (!found){
    //     all_detections.poses.push_back(old_pose);
    //   }
    // }

    all_detections.header.stamp = rclcpp::Node::now(); 
    all_detections.header.frame_id = "base_link"; 
    base_link_det_pub->publish(all_detections);

    previous_all_detections = all_detections;

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
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr base_link_det_pub;
  geometry_msgs::msg::PoseArray previous_all_detections;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  float dist_thresh;
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