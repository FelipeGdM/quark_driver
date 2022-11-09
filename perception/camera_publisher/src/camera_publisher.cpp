#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// #include "camera_reader.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/header.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("camera_publisher");
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("raw_image", 1);

  rclcpp::Rate rate(60);
  Mat image;
  VideoCapture cap(0);
  cout << "Initializing" << endl;

  if(!cap.isOpened()){
    cout << "Not open" << endl;
    return -1;
  }

  std::shared_ptr<sensor_msgs::msg::Image> msg;

  while(rclcpp::ok()){
    cap >> image;

    if (image.empty()) {
        cout << "Image is empty" << endl;
        continue;
    }

    msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
    pub.publish(msg);
    

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
  