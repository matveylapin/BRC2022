import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
from copy import deepcopy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import ExtrapolationException, ConnectivityException, LookupException

MAX_ANGULAR_SPEED = 1.5
MIN_ANGULAR_SPEED = 0.1
ANGULAR_TOLERANCE = 0.05
ANGULAR_P = 1.2
MAX_LINEAR_SPEED = 0.6
MIN_LINEAR_SPEED = 0.2
LINEAR_TOLERANCE = 0.2
LINEAR_P = 1.0


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


def normilize_angle(_a):
    a = _a
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class GoalSubscriber(Node):

    def __init__(self):
        global server
        super().__init__('GOAL_subscriber')
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile_sensor_data)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            qos_profile_sensor_data)

        self.mapa = None
        self.current_position = None
        self.target_position = None
        self.angle_reached = False
        self.dist_reached = False
        self.target_angle_reached = False

    def map_callback(self, data):
        self.mapa = deepcopy(data)
        self.update_position()

    def goal_pose_callback(self, data):
        self.target_position = [
            data.pose.position.x,
            data.pose.position.y,
            euler_from_quaternion(data.pose.orientation)[2]
        ]
        self.angle_reached = False
        self.dist_reached = False
        self.target_angle_reached = False

    def update_position(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            self.current_position = [
                t.transform.translation.x,
                t.transform.translation.y,
                euler_from_quaternion(t.transform.rotation)[2]
            ]
            self.update_velocity()

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('waiting for map transform')

    def update_velocity(self):
        if self.target_position == None:
            return
        movement = Twist()

        if not self.angle_reached:
            movement = self.turn_to_point()
        elif not self.dist_reached:
            movement = self.go_to_point()
        elif not self.target_angle_reached:
            movement = self.turn_to_target_angle()
        else:
            self.target_position = None

        self.publisher.publish(movement)

    def turn_to_point(self):
        movement = Twist()

        diff = normilize_angle(-self.current_position[2] + math.atan2(
            - self.current_position[1] + self.target_position[1],
            - self.current_position[0] + self.target_position[0]))

        if abs(diff) < ANGULAR_TOLERANCE:
            self.angle_reached = True
            return movement

        mul = diff / abs(diff)
        movement.angular.z = mul * \
            min(MAX_ANGULAR_SPEED, max(MIN_ANGULAR_SPEED, abs(diff) * ANGULAR_P))

        return movement

    def go_to_point(self):
        movement = Twist()

        d_diff = math.sqrt((self.current_position[1] - self.target_position[1]) ** 2 +
                           (self.current_position[0] - self.target_position[0]) ** 2)

        if d_diff < LINEAR_TOLERANCE:
            self.dist_reached = True
            return movement

        movement.linear.x = min(MAX_LINEAR_SPEED, max(
            MIN_LINEAR_SPEED, d_diff * LINEAR_P))

        return movement

    def turn_to_target_angle(self):
        movement = Twist()

        diff = normilize_angle(
            -self.current_position[2] + self.target_position[2])

        if abs(diff) < ANGULAR_TOLERANCE:
            self.target_angle_reached = True
            return movement

        mul = diff / abs(diff)
        movement.angular.z = mul * \
            min(MAX_ANGULAR_SPEED, max(MIN_ANGULAR_SPEED, abs(diff) * ANGULAR_P))

        return movement

    def timer_callback(self):
        self.update_position()


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = GoalSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
