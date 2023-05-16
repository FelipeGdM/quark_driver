#!/usr/bin/python3.8

from typing import List
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data

from matplotlib import pyplot as plt

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray, Pose

# https://nilesh0109.medium.com/camera-image-perspective-transformation-to-different-plane-using-opencv-5e389dd56527

CAMERA = "front/left"

SIZE = 1000

def get_inverse_pespective(perspective_matrix: np.array) -> np.array:
    """
    This method calculates the inverse of prespective matrix by homography.
    - Takes 4 random points on the floor plane(destination_plane) and calculates the corresponding points
    on the camera image plane(src_plane) using perspective matrix.
    - Calculates the Homography matrix to map any point in image plane to floor plane.
    Parameters
    ----------
    perspective_matrix: 3 x 4 camera prespective matrix to convert 3d homogeneous world coordinates to
    2d homogeneous camera coordinates.
    Returns
    ----------
    3x3 homography matrix for moving from 2d homogeneous image plane to world floor plane(at z=0)

    """
    # Take 5 homogenous points on the floor(Unit is in Meters)
    pts_dst = np.array(
        [
            [-0.5, 0.15, -0.05, 1], 
            [+0.5, 0.15, -0.05, 1], 
            [-0.5, 0.15, 1, 1], 
            [+0.5, 0.15, 1, 1]
        ]
    )

    # pts_dst = np.array(
    #     [
    #         [0.1, -0.5, 1, 1], 
    #         [0.1, 0.5, 1, 1], 
    #         [1, -0.5, -0.5, 1], 
    #         [1, 0.5, -0.5, 1]
    #     ]
    # )
    # pts_dst = pts_dst_tmp.copy()
    # pts_dst[:,0] = -pts_dst_tmp[:,1]
    # pts_dst[:,1] = -pts_dst_tmp[:,2]
    # pts_dst[:,2] = pts_dst_tmp[:,0]
    # pts_dst[:,3] = pts_dst_tmp[:,3]
    # Obtain respective homogenous points on the image plane
    # P = np.empty((4, 4))
    # P[:3, :] = perspective_matrix
    # P[3, :] = [0, 0, 0, 1]

    # P_inv = np.linalg.inv(P)

    # A = P_inv @ np.array([0,0,1], )
    pts_src = (perspective_matrix.reshape(3,4) @ pts_dst.T).T + np.array([320, 240, 0])

    print(pts_src)
    # convert homogenous coordinates to cartesian coorndinates
    pts_src_cart = np.array([[x / w, y / w] for x, y, w in pts_src])
    pts_dst_cart = np.array([[0, 0],
                            [0, SIZE], 
                            [SIZE, 0],
                            [SIZE, SIZE]])/4 + np.array([0, SIZE*0.6])

    # find the 3x3 Homography Matrix for transforming image plane to floor plane
    H, status = cv2.findHomography(pts_src_cart, pts_dst_cart)
    return H


# def project_to_floor(image_coordinates: List[int], H: np.array) -> List[int]:
#     """
#     This method takes the Homography matrix and the 2d image cartesian coordinates. It returns the (x, y)
#     cartesian coordinates in 3d cartesian world coordinates on floor plane(at z=0). Notice that z coordinate is omitted
#     here and added inside the tracking funtion.

#     Parameters
#     ----------
#     image_coordinates: 2d pixel coordinates (x,y)
#     h: 3x3 Homography matrix np.array[3x3]
#     Returns
#     ----------
#     floor_coordinates: List of x, y coordinates in 3d world of same pixel on floor plane i.e. (x,y,z) Considering z=0 and
#     ommitted here.
#     """
#     # adding 1 for homogenous coordinate system
#     x, y, w = H @ np.array([[*image_coordinates, 1]]).T
#     return [x / w, y / w]


class ParkingCamera(Node):
    bridge = CvBridge()

    def __init__(self):
        super().__init__("parking_camera")
        self.subscription_img = self.create_subscription(
            Image,
            f"/stereo_camera/{CAMERA}/image_rect",
            self.image_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            f"/stereo_camera/{CAMERA}/camera_info",
            self.camera_info_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.floor_pub = self.create_publisher(Image, "/camera/floor_projection", 1)

        self.camera_info = None
        self.H = None

        self.get_logger().info("Ready to receive")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info == None:
            self.H = get_inverse_pespective(msg.p.reshape(3,4))
            print(self.H)
        self.camera_info = msg

    def image_callback(self, msg):

        if self.camera_info == None:
            self.get_logger().error(
                "Didn't received camera info yet, skipping image callback"
            )
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        im_dst = cv2.warpPerspective(cv_image, self.H, (SIZE, SIZE))
        cv2.imshow("topster", im_dst)
        cv2.waitKey(1)

        self.floor_pub.publish(self.bridge.cv2_to_imgmsg(im_dst, "bgr8"))
        # cv2.imshow("Camer sub", display_img)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    parking_camera = ParkingCamera()

    rclpy.spin(parking_camera)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    parking_camera.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
