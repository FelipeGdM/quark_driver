import cv2
import rclpy
from .yolov7 import Yolov7
from rclpy.node import Node
from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data

from matplotlib import pyplot as plt

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose

class ImageSubscriber(Node):
    bridge = CvBridge()

    def __init__(self):
        super().__init__('cone_detector')
        self.subscription = self.create_subscription(Image, '/stereo_camera/fused/image_raw', self.image_callback, qos_profile=qos_profile_sensor_data)
        self.detections_pub = self.create_publisher(Image, '/camera/detections', 1)
        self.cone_centers_pub = self.create_publisher(PoseArray, '/camera/cone_centers', 1)
        
        #self.yolov7 = Yolov7("/home/tocoquinho/repositories/ros2_ws/src/quark_driver/perception/cone_detection_python/params/tiny_cone_weights.pt", 0.4, 0.25, 480, None)
        self.yolov7 = Yolov7("/home/quark/Documents/yolov7/400_epochs_tiny.pt", 0.6, 0.5, 1280, None)
        print("Ready to receive")

    def image_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        display_img, predictions = self.yolov7.detect([cv_image])
        predictions.sort(key=lambda x: x[1]) #sort by precision

        centers = PoseArray()
        centers.header = msg.header
        if len(predictions):
            for (rects, conf) in predictions:
                rect_center = Pose()
                rect_center.position.x = float(rects[2] + rects[0])/2
                rect_center.position.y = float(rects[3] + rects[1])/2
                centers.poses.append(rect_center)

                # if len(centers.poses) >= 4:
                #     break

        self.detections_pub.publish(self.bridge.cv2_to_imgmsg(display_img, "bgr8"))
        self.cone_centers_pub.publish(centers)

        # cv2.imshow("Camer sub", display_img)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    cone_detector = ImageSubscriber()

    rclpy.spin(cone_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cone_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
