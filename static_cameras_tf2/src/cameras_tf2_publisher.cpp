#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher : public rclcpp::Node
{
public:
    explicit StaticFramePublisher(std::string frameId, float* translation, tf2::Matrix3x3 rotationMatrix)
    : Node("tf2_publisher")
    {
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms once at startup
        this->make_transforms(frameId, translation, rotationMatrix);
    }

private:
    void make_transforms(std::string frameId, float* translation, tf2::Matrix3x3 rotationMatrix)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = frameId;

        t.transform.translation.x = translation[0];
        t.transform.translation.y = translation[1];
        t.transform.translation.z = translation[2];
        
        tf2::Quaternion q;
        rotationMatrix.getRotation(q);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
    auto logger = rclcpp::get_logger("logger");
    rclcpp::init(argc, argv);

    std::cout << "Publishing cameras static transform" << std::endl;

    tf2::Matrix3x3 rotationMatrix;
    float* translation;

    //Front camera
    rotationMatrix.setValue( 0, 0, 1,
                            -1, 0, 0,
                             0, -1, 0);
    translation = new float[3] {0.09 , 0.03 , 0};
    std::make_shared<StaticFramePublisher>("front_camera", translation, rotationMatrix);
    std::cout << "Published front" << std::endl;

    //Left camera
    rotationMatrix.setValue(1, 0, 0,
                            0, 0, 1,
                            0, -1, 0);
    translation = new float[3] {-0.03, 0.08, 0};
    std::make_shared<StaticFramePublisher>("left_camera", translation, rotationMatrix);
    std::cout << "Published left" << std::endl;

    //Right camera
    rotationMatrix.setValue(-1, 0, 0,
                            0, 0, -1,
                            0, -1, 0);
    translation = new float[3] {0.03, -0.08, 0};
    std::make_shared<StaticFramePublisher>("right_camera", translation, rotationMatrix);
    std::cout << "Published right" << std::endl;

    //Back camera
    rotationMatrix.setValue(0, 0, -1,
                            1, 0, 0,
                            0, -1, 0);
    translation = new float[3] {-0.09, -0.03, 0};
    std::make_shared<StaticFramePublisher>("back_camera", translation, rotationMatrix);
    std::cout << "Published back" << std::endl;

    std::cout << "Done publishing transforms" << std::endl;

    rclcpp::shutdown();
    return 0;
}