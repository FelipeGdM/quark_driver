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
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/header.hpp"

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

class ConePoseEstimator : public rclcpp::Node
{
public:
  ConePoseEstimator()
      : Node("exact_time_subscriber")
  {
    left_image_sub.subscribe(this, "/stereo_camera/left/image_depth", rmw_qos_profile_sensor_data);
    front_image_sub.subscribe(this, "/stereo_camera/front/image_depth", rmw_qos_profile_sensor_data);
    right_image_sub.subscribe(this, "/stereo_camera/right/image_depth", rmw_qos_profile_sensor_data);
    // back_image_sub.subscribe(this, "/stereo_camera/back/image_depth", rmw_qos_profile_sensor_data);

    left_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_pose/left", 1); 
    front_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_pose/front", 1); 
    right_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_pose/right", 1); 
    back_det_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_pose/back", 1); 

    left_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/stereo_camera/left/left/camera_info", 1, std::bind(&ConePoseEstimator::left_cam_info_callback, this, _1));
    front_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/stereo_camera/front/left/camera_info", 1, std::bind(&ConePoseEstimator::front_cam_info_callback, this, _1));
    right_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/stereo_camera/right/left/camera_info", 1, std::bind(&ConePoseEstimator::right_cam_info_callback, this, _1));
    back_cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/stereo_camera/back/left/camera_info", 1, std::bind(&ConePoseEstimator::back_cam_info_callback, this, _1));

    detections_sub.subscribe(this, "/camera/cone_centers");

    sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::msg::PoseArray, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(detections_sub, front_image_sub, left_image_sub, right_image_sub, 3);
    sync_->registerCallback(std::bind(&ConePoseEstimator::images_callback, this, _1, _2, _3, _4));
  }

private:
  void images_callback(const geometry_msgs::msg::PoseArray::ConstSharedPtr& detections, const sensor_msgs::msg::Image::ConstSharedPtr& front_img_msg, const sensor_msgs::msg::Image::ConstSharedPtr& left_img_msg, const sensor_msgs::msg::Image::ConstSharedPtr& right_img_msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: bananas");
    cv_bridge::CvImagePtr front_img = cv_bridge::toCvCopy(front_img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv_bridge::CvImagePtr left_img = cv_bridge::toCvCopy(left_img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv_bridge::CvImagePtr right_img = cv_bridge::toCvCopy(right_img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    // cv_bridge::CvImagePtr back_img = cv_bridge::toCvCopy(back_img_msg, sensor_msgs::image_encodings::TYPE_32FC1);

    int h = front_img->image.rows;
    int w = front_img->image.cols;

    geometry_msgs::msg::PoseArray left_detections;
    geometry_msgs::msg::PoseArray front_detections;
    geometry_msgs::msg::PoseArray right_detections;
    geometry_msgs::msg::PoseArray back_detections;

    geometry_msgs::msg::Pose pose;
    if(sizeof(detections->poses)){
      for(int i = 0; i < sizeof(detections->poses)/sizeof(detections->poses[0]); i++){
        pose = detections->poses[i];

        int img_num = pose.position.x / w;
        pose.position.x = (float)((int)pose.position.x % (int)w);

        float z;
        switch (img_num)
        {
        case 0:
          if (left_cam_info == nullptr) {
            RCUTILS_LOG_WARN("No left camera info, skipping conversion");
            return;
          }

          z = left_img->image.at<float>(pose.position.x, pose.position.y);
          pose.position.x = (pose.position.x - left_cam_info->k[2]) * z/left_cam_info->k[0];
          pose.position.y = (pose.position.y - left_cam_info->k[5]) * z/left_cam_info->k[4];
          left_detections.poses.push_back(pose);
          break;

        case 1:
          if (front_cam_info == nullptr) {
            RCUTILS_LOG_WARN("No front camera info, skipping conversion");
            break;
          }

          z = front_img->image.at<float>(pose.position.x, pose.position.y);
          pose.position.x = (pose.position.x - front_cam_info->k[2]) * z/front_cam_info->k[0];
          pose.position.y = (pose.position.y - front_cam_info->k[5]) * z/front_cam_info->k[4];
          front_detections.poses.push_back(pose);
          break;
        
        case 2:
          if (right_cam_info == nullptr) {
            RCUTILS_LOG_WARN("No right camera info, skipping conversion");
            break;
          }

          z = right_img->image.at<float>(pose.position.x, pose.position.y);
          pose.position.x = (pose.position.x - right_cam_info->k[2]) * z/right_cam_info->k[0];
          pose.position.y = (pose.position.y - right_cam_info->k[5]) * z/right_cam_info->k[4];
          right_detections.poses.push_back(pose);
          break;
        
        case 3:
          // if (back_cam_info == nullptr) {
          //   RCUTILS_LOG_WARN("No back camera info, skipping conversion");
          //   break;
          // }
          // z = back_img->image.at<float>(pose.position.x, pose.position.y);
          // pose.position.x = (pose.position.x - back_cam_info->k[2]) * z/back_cam_info->k[0];
          // pose.position.y = (pose.position.y - back_cam_info->k[5]) * z/back_cam_info->k[4];
          // back_detections.poses.push_back(pose);
          break;
        
        default:
          break;
        }
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

  message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> front_image_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> back_image_sub;
  message_filters::Subscriber<geometry_msgs::msg::PoseArray> detections_sub;
  std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::msg::PoseArray, sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync_;

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
  rclcpp::spin(std::make_shared<ConePoseEstimator>());
  rclcpp::shutdown();

  return 0;
}