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
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/header.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

using namespace std;
using namespace cv;

static sensor_msgs::msg::CameraInfo::SharedPtr left_cam_info;
static sensor_msgs::msg::CameraInfo::SharedPtr front_cam_info;
static sensor_msgs::msg::CameraInfo::SharedPtr right_cam_info;
static sensor_msgs::msg::CameraInfo::SharedPtr back_cam_info;

static geometry_msgs::msg::PoseArray::SharedPtr detections_msg;
static stereo_msgs::msg::DisparityImage::SharedPtr left_img_msg;
static stereo_msgs::msg::DisparityImage::SharedPtr front_img_msg;
static stereo_msgs::msg::DisparityImage::SharedPtr right_img_msg;
static stereo_msgs::msg::DisparityImage::SharedPtr back_img_msg;

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

    left_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/stereo_camera/left/left/camera_info", 1, std::bind(&ConePoseEstimator::left_cam_info_callback, this, _1));
    front_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/stereo_camera/front/left/camera_info", 1, std::bind(&ConePoseEstimator::front_cam_info_callback, this, _1));
    right_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/stereo_camera/right/left/camera_info", 1, std::bind(&ConePoseEstimator::right_cam_info_callback, this, _1));
    back_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/stereo_camera/back/left/camera_info", 1, std::bind(&ConePoseEstimator::back_cam_info_callback, this, _1));

    detections_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/camera/cone_centers", 1, std::bind(&ConePoseEstimator::detections_callback, this, _1));
    left_image_sub = this->create_subscription<stereo_msgs::msg::DisparityImage>("/stereo_camera/left/left/disparity", 1, std::bind(&ConePoseEstimator::left_img_callback, this, _1));
    front_image_sub = this->create_subscription<stereo_msgs::msg::DisparityImage>("/stereo_camera/front/left/disparity", 1, std::bind(&ConePoseEstimator::front_img_callback, this, _1));
    right_image_sub = this->create_subscription<stereo_msgs::msg::DisparityImage>("/stereo_camera/right/left/disparity", 1, std::bind(&ConePoseEstimator::right_img_callback, this, _1));
    back_image_sub = this->create_subscription<stereo_msgs::msg::DisparityImage>("/stereo_camera/back/left/disparity", 1, std::bind(&ConePoseEstimator::back_img_callback, this, _1));
  
    RCLCPP_INFO(this->get_logger(), "Ready to receive images and detections!!");
  }

// Intrinsic camera matrix for the raw (distorted) images.
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]
// Projects 3D points in the camera coordinate frame to 2D pixel
// coordinates using the focal lengths (fx, fy) and principal point
// (cx, cy).

  void spin(){
    if(  detections_msg != nullptr
      && front_img_msg != nullptr 
      && right_img_msg != nullptr 
      && left_img_msg != nullptr 
      // && back_img_msg != nullptr 
      ){
      estimate_poses();

      reset_msgs();
    }
  }

private:
  void reset_msgs(){
    left_img_msg = nullptr;
    front_img_msg = nullptr;
    right_img_msg = nullptr;
    back_img_msg = nullptr;
    detections_msg = nullptr;
  }

  void estimate_poses()
  {
    RCLCPP_INFO(this->get_logger(), "I heard: bananas");
    auto front_img = cv_bridge::toCvShare(front_img_msg->image, front_img_msg, "8UC1");
    auto left_img = cv_bridge::toCvShare(left_img_msg->image, left_img_msg, "8UC1");
    auto right_img = cv_bridge::toCvShare(right_img_msg->image, right_img_msg, "8UC1");
    // cv_bridge::CvImageConstPtr back_img = cv_bridge::toCvShare(back_img_msg->image, back_img_msg, "8UC1");

    int h = front_img->image.rows;
    int w = front_img->image.cols;

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

      int disp;
      float z;
      float cone_height = 0.07; //m
      switch (img_num)
      {
      case 0:
        if (left_cam_info == nullptr) {
          RCUTILS_LOG_WARN("No left camera info, skipping conversion");
          return;
        }

        // disp = left_img->image.at<int>(pose.position.x, pose.position.y);
        z = 0.4;
                disp = right_img->image.at<uint8_t>(pose.position.x, pose.position.y);
        z = disp == 0 ? z = 5 : (right_img_msg->t * right_img_msg->f / (float)disp);
        pose.position.x = (pose.position.x - left_cam_info->k[2]) * z/left_cam_info->k[0];
        pose.position.y = ((pose.position.y - left_cam_info->k[5]) * z/left_cam_info->k[4]) + cone_height/2;
        pose.position.z = z;
        left_detections.poses.push_back(pose);
        break;

      case 1:
        if (front_cam_info == nullptr) {
          RCUTILS_LOG_WARN("No front camera info, skipping conversion");
          break;
        }

        // disp = front_img->image.at<int>(pose.position.x, pose.position.y);
        z = 0.4;
                disp = right_img->image.at<uint8_t>(pose.position.x, pose.position.y);
        z = disp == 0 ? z = 5 : (right_img_msg->t * right_img_msg->f / (float)disp);
        pose.position.x = (pose.position.x - front_cam_info->k[2]) * z/front_cam_info->k[0];
        pose.position.y = ((pose.position.y - front_cam_info->k[5]) * z/front_cam_info->k[4]) + cone_height/2;
        pose.position.z = z;
        front_detections.poses.push_back(pose);
        break;
      
      case 2:
        if (right_cam_info == nullptr) {
          RCUTILS_LOG_WARN("No right camera info, skipping conversion");
          break;
        }

        cv::imshow("Disp", right_img->image);
        cv::waitKey(1);

        disp = right_img->image.at<uint8_t>(pose.position.x, pose.position.y);
        z = disp == 0 ? z = 5 : (right_img_msg->t * right_img_msg->f / (float)disp);
        // z = 0.4;
        RCLCPP_INFO_STREAM(this->get_logger(), "z: " << z << " | disp: " << disp);

        pose.position.x = (pose.position.x - right_cam_info->k[2]) * z/right_cam_info->k[0];
        pose.position.y = ((pose.position.y - right_cam_info->k[5]) * z/right_cam_info->k[4]) + cone_height/2;
        pose.position.z = z;
        right_detections.poses.push_back(pose);
        break;
      
      case 3:
        if (back_cam_info == nullptr) {
          RCUTILS_LOG_WARN("No back camera info, skipping conversion");
          break;
        }
        // disp = back_img->image.at<int>(pose.position.x, pose.position.y);
        // pose.position.x = (pose.position.x - back_cam_info->k[2]) * z/back_cam_info->k[0];
        // pose.position.y = ((pose.position.y - back_cam_info->k[5]) * z/back_cam_info->k[4]) + cone_height/2;
        // pose.position.z = z;
        // back_detections.poses.push_back(pose);
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

  void left_cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info) const 
  {
    left_cam_info = cam_info;
  }
  void front_cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info) const 
  {
    front_cam_info = cam_info;
  }
  void right_cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info) const 
  {
    right_cam_info = cam_info;
  }
  void back_cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info) const 
  {
    back_cam_info = cam_info;
  }

  void detections_callback(const geometry_msgs::msg::PoseArray::SharedPtr detections) const 
  {
    detections_msg = detections;
  }
  void left_img_callback(const stereo_msgs::msg::DisparityImage::SharedPtr img) const 
  {
    left_img_msg = img;
  }
  void front_img_callback(const stereo_msgs::msg::DisparityImage::SharedPtr img) const 
  {
    front_img_msg = img;
  }
  void right_img_callback(const stereo_msgs::msg::DisparityImage::SharedPtr img) const 
  {
    right_img_msg = img;
  }
  void back_img_callback(const stereo_msgs::msg::DisparityImage::SharedPtr img) const 
  {
    back_img_msg = img;
  }

  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr left_image_sub;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr right_image_sub;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr front_image_sub;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr back_image_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detections_sub;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr left_det_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr front_det_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr right_det_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr back_det_pub;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_cam_info_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr front_cam_info_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_cam_info_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr back_cam_info_sub;
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