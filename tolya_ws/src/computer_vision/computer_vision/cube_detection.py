import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import math


cube = ['f', 'r', 'b', 'l', 'u', 'd']
marker_size = 0.09
cube_side_rotations = {
    'f': np.array([math.pi / 2, 0.0, math.pi]),
    'r': np.array([math.pi / 2, math.pi / 2, math.pi]),
    'b': np.array([math.pi / 2, math.pi, math.pi]),
    'l': np.array([math.pi / 2, -math.pi / 2, math.pi]),
    'u': np.array([0.0, 0.0, math.pi]),
    'd': np.array([math.pi, 0.0, math.pi])
}

distortion_coefficients = np.array(
    [0.041074563385132476, -0.23317633985075079, 0.002736896865846102, 0.0013594533508533232, 0.24301538197366987])
matrix_coefficients = np.array(
    [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])

camera_rotation = np.array([0.0, 0.0, 0.0])
camera_translation = np.array([0.0, 0.0, 0.0])


def eul2rot(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


for k, v in cube_side_rotations.items():
    cube_side_rotations[k] = eul2rot(v)

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('cube_detection_publisher')
        self.publisher_ = self.create_publisher(Image, '/cube_detection/result', 10)

    def publish_image(self, msg):
        self.publisher_.publish(msg)


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('cube_detection_subsciber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.br = CvBridge()
        self.image_publisher = ImagePublisher()
        self.subscription

    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        markers_corners, ids, rejected = cv.aruco.detectMarkers(gray, arucoDict)
        cv.aruco.drawDetectedMarkers(frame, markers_corners, ids)

        cube_point_n = 0
        cube_point = np.array([0.0, 0.0, 0.0])
        cube_rotation = np.array([0.0, 0.0, 0.0])

        rm = None

        if np.all(ids is not None):
            for corners, marker_id in zip(markers_corners, ids):
                if marker_id <= 5:
                    rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, marker_size, matrix_coefficients,
                                                                                distortion_coefficients)
                    rv = rvec[0][0]
                    tv = tvec[0][0]
                    rm, _ = cv.Rodrigues(rv)
                    tv += rm.dot(np.array([0, 0, -0.05]))
                    rm = rm.dot(cube_side_rotations[cube[marker_id[0]]])

                    cube_point += tv
                    cube_point_n += 1
                    cube_rotation = rm

        if cube_point_n != 0:
            cube_point /= cube_point_n

            result, _ = cv.projectPoints(cube_point, camera_rotation, camera_translation, matrix_coefficients,
                                        distortion_coefficients)
            cv.drawFrameAxes(frame, matrix_coefficients,
                            distortion_coefficients, cube_rotation, cube_point, 0.1)
            cv.circle(frame, result[0][0].round().astype(int), 1, (0, 0, 255), 5)

        self.image_publisher.publish_image(self.br.cv2_to_imgmsg(frame))

    def stop(self):
        self.image_publisher.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.stop()
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()