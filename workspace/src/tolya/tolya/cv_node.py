import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2 as cv
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import numpy as np
import math
from json import loads


arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)

distortion = None
camera_matrix = None


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

    R = np.dot(R_x, np.dot(R_y, R_z))

    return R


def rotationMatrixToQuaternion1(m):
    #q0 = qw
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if (t > 0):
        t = np.sqrt(t + 1)
        q[3] = 0.5 * t
        t = 0.5/t
        q[0] = (m[2, 1] - m[1, 2]) * t
        q[1] = (m[0, 2] - m[2, 0]) * t
        q[2] = (m[1, 0] - m[0, 1]) * t

    else:
        i = 0
        if (m[1, 1] > m[0, 0]):
            i = 1
        if (m[2, 2] > m[i, i]):
            i = 2
        j = (i+1) % 3
        k = (j+1) % 3

        t = np.sqrt(m[i, i] - m[j, j] - m[k, k] + 1)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k, j] - m[j, k]) * t
        q[j] = (m[j, i] + m[i, j]) * t
        q[k] = (m[k, i] + m[i, k]) * t

    return q


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('cube_detection_subsciber')
        self.publisher = self.create_publisher(
            Image, '/cv_node/result', qos_profile_sensor_data)
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data)

        self.br = CvBridge()
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.iter = 0

        self.declare_parameter('markers', '{\}')
        self.markers = loads(self.get_parameter(
            'markers').get_parameter_value().string_value)
        self.process_markers()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def process_markers(self):
        for k in self.markers.keys():
            try:
                self.markers[k]["rm"] = eul2rot(np.array(self.markers[k]["r"]))
                self.markers[k]["rmt"] = self.markers[k]["rm"].dot(
                    -np.array(self.markers[k]["t"]))
            except KeyError:
                pass

    def camera_info_callback(self, msg):
        global distortion, camera_matrix

        distortion = np.array(msg.d)
        camera_matrix = np.array(msg.k).reshape(3, 3)

    def image_callback(self, msg):
        global camera_matrix, distortion
        if type(camera_matrix) != np.ndarray:
            return
        if type(distortion) != np.ndarray:
            return
        self.iter += 1
        if self.iter == 2:
            self.iter = 0
            return
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        frame = self.detect_markers(frame)

        self.publisher.publish(self.br.cv2_to_imgmsg(frame))

    def detect_markers(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        markers_corners, ids, rejected = cv.aruco.detectMarkers(
            gray, arucoDict)
        cv.aruco.drawDetectedMarkers(frame, markers_corners, ids)

        if np.all(ids is not None):
            points = []

            for corners, marker_id in zip(markers_corners, ids):
                if str(marker_id[0]) in self.markers.keys():
                    try:
                        marker = self.markers[str(marker_id[0])]
                        rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(
                            corners, marker["size"], camera_matrix, distortion)
                        rv = rvec[0][0]
                        tv = tvec[0][0]
                        rm, _ = cv.Rodrigues(rv)
                        # x - red, y - green, z - blue
                        tv += rm.dot(marker["rmt"])
                        rm = rm.dot(marker["rm"])
                        points.append([tv, rm])
                    except KeyError:
                        pass

            avg_t = np.array([0.0, 0.0, 0.0])
            avg_rm = None
            avg_c = 0

            for point in points:
                # вот тут можно как-то отфильтровать
                avg_t += point[0]
                if avg_c == 0:
                    avg_rm = point[1]
                else:
                    avg_rm += point[1]
                avg_c += 1

            if avg_c > 0:
                pose = [avg_t / avg_c, avg_rm / avg_c]

                if avg_c > 0:
                    cv.drawFrameAxes(frame, camera_matrix,
                                     distortion, pose[1], pose[0], 0.1)

                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_frame'
                t.child_frame_id = 'cube_in_camera'

                t.transform.translation.x = pose[0][0]
                t.transform.translation.y = pose[0][1]
                t.transform.translation.z = pose[0][2]

                q = rotationMatrixToQuaternion1(pose[1])
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                self.tf_broadcaster.sendTransform(t)

                try:
                    # жесть костыль кстати
                    t = self.tf_buffer.lookup_transform(
                        'map',
                        'cube_in_camera',
                        rclpy.time.Time())
                    t.child_frame_id = 'cube'
                    self.tf_broadcaster.sendTransform(t)
                except:
                    pass

        return frame


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
