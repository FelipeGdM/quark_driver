import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

from sensor_msgs.msg import Image

class MinimalSubscriber(Node):
    bridge = CvBridge()

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Image, 'raw_image', self.image_callback, 1)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        cv2.imshow("Camer sub", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()