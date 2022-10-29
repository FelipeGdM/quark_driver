#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

using namespace cv;
using namespace std;
using std::placeholders::_1;

class CameraSubscriber : public rclcpp::Node
{
  public:
    CameraSubscriber(): Node("camera_subscriber")
    {
      _subscription = this->create_subscription<sensor_msgs::msg::Image>("raw_image", 10, bind(&CameraSubscriber::image_callback, this, _1));
      namedWindow("Webcam");
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      imshow("Webcam", cv_ptr->image);
      waitKey(1);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}