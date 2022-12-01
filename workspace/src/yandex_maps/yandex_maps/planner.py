import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
from copy import deepcopy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import ExtrapolationException, ConnectivityException, LookupException


def euler_from_quaternion(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


def coords_to_point(mapa, xy):
    origin = mapa.info.origin
    theta = euler_from_quaternion(origin.orientation)[2]
    info = mapa.info

    R = np.array([[math.cos(theta), math.sin(theta)],
                  [-math.sin(theta), math.cos(theta)]])

    R = np.linalg.inv(R)

    # [x;y] = R * [px * resolution ; py * resolution] + [x0,y0]
    # [px; py] = (invR * ([x; y] - [x0, y0])) / resolution
    return ((R.dot(xy - np.array([origin.position.x, origin.position.y]))) / info.resolution).round().astype(int)


def point_to_coords(mapa, xy):
    origin = mapa.info.origin
    theta = euler_from_quaternion(origin.orientation)[2]
    info = mapa.info

    R = np.array([[math.cos(theta), math.sin(theta)],
                  [-math.sin(theta), math.cos(theta)]])
    return R.dot(info.resolution * xy) + np.array([origin.position.x, origin.position.y])


class PlannerNode(Node):

    def __init__(self):
        super().__init__('PlannerNode')
        self.path_publisher = self.create_publisher(
            Path, '/path', 10)

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            qos_profile_sensor_data)

        self.map_subsriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def goal_pose_callback(self, data):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            current_position = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                euler_from_quaternion(t.transform.rotation)[2]
            ])

            target_position = np.array([
                data.pose.position.x,
                data.pose.position.y,
                euler_from_quaternion(data.pose.orientation)[2]
            ])

            map_point = coords_to_point(self.map, current_position[:2])
            map_target_point = coords_to_point(self.map, target_position[:2])

            self.get_logger().info('Building path from ' +
                                   str(map_point) + ' to ' + str(map_target_point))
            map_path = self.find_path(self.map, map_point, map_target_point)
            if len(map_path) == 0:
                self.get_logger().warn('No path')
                return
            self.get_logger().info('Found path')

            path = Path()
            path.header.frame_id = "map"
            poses = []

            for i in range(len(map_path)):
                pose = PoseStamped()
                pose.header.frame_id = "map"

                point = point_to_coords(self.map, map_path[i])

                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]

                if i < len(map_path) - 1:
                    next_point = point_to_coords(self.map, map_path[i + 1])

                    rotation = math.atan2(
                        next_point[1] - point[1], next_point[0] - point[0])
                    q = quaternion_from_euler(0, 0, rotation)
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                else:
                    pose.pose.orientation = data.pose.orientation
                poses.append(pose)

            path.poses = poses
            self.path_publisher.publish(path)

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('waiting for map transform')

    def find_path(self, mapa, p1, p2):
        points_per_pixel = 0.5
        points = int(points_per_pixel * ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5)

        def intermediates(p1, p2, nb_points=8):
            x_spacing = (p2[0] - p1[0]) / (nb_points + 1)
            y_spacing = (p2[1] - p1[1]) / (nb_points + 1)

            return [[p1[0] + i * x_spacing, p1[1] + i * y_spacing]
                    for i in range(1, nb_points+1)]

        return list(map(np.array, intermediates(p1, p2, points)))

    def map_callback(self, data):
        self.map = deepcopy(data)


def main(args=None):
    rclpy.init(args=args)
    planner_node = PlannerNode()

    rclpy.spin(planner_node)

    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
