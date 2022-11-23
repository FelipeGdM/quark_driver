#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <sys/time.h>
#include "yolo.h"

using namespace cv;
using namespace std;
using std::placeholders::_1;

class CameraSubscriber : public rclcpp::Node
{
  public:
    CameraSubscriber(): Node("camera_subscriber"), yolov7()
    {
      _subscription = this->create_subscription<sensor_msgs::msg::Image>("raw_image", 1, bind(&CameraSubscriber::image_callback, this, _1));
      yolov7.load(360);
      namedWindow("Yolov7");
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

      std::vector<Object> objects;
  
      struct timeval start, end;

      gettimeofday(&start, NULL);
      yolov7.detect(image, objects, 0.5);
      gettimeofday(&end, NULL);
    
      double time_taken;
    
      time_taken = (end.tv_sec - start.tv_sec) * 1e6;
      time_taken = (time_taken + (end.tv_usec - 
                                start.tv_usec)) * 1e-3;
    
      cout << "Time taken by program is : " << fixed
          << time_taken << setprecision(6);
      cout << " msec" << endl;

      yolov7.draw(image, objects);

      imshow("Yolov7", image);
      waitKey(1);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;
    YoloV7 yolov7;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}