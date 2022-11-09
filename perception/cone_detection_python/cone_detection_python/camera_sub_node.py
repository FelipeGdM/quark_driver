import cv2
import rclpy
from .yolov7 import Yolov7
from rclpy.node import Node
from cv_bridge import CvBridge

def main(args=None):
    rclpy.init(args=args)

    cone_detector = ImageSubscriber()

from matplotlib import pyplot as plt

from sensor_msgs.msg import Image

class ImageSubscriber(Node):
    bridge = CvBridge()

    def __init__(self):
        super().__init__('cone_detector')
        self.subscription = self.create_subscription(Image, 'raw_image', self.image_callback, 1)
        self.yolov7 = Yolov7("/home/tocoquinho/repositories/ros2_ws/src/quark_driver/perception/cone_detection_python/params/tiny_cone_weights.pt", 0.4, 0.25, 480, None)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        display_img = self.yolov7.detect([cv_image])

        cv2.imshow("Camer sub", display_img)
        cv2.waitKey(1)


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