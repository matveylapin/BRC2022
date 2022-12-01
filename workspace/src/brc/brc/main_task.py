import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
from copy import deepcopy
from time import sleep
import threading
from json import loads

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import ExtrapolationException, ConnectivityException, LookupException


DIST_TOLERANCE = 0.05
ANGLE_TOLERANCE = 0.05


def normilize_angle(_a):
    a = _a
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


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
    s_roll = np.sin(roll/2)
    c_roll = np.cos(roll/2)
    s_pitch = np.sin(pitch/2)
    c_pitch = np.cos(pitch/2)
    s_yaw = np.sin(yaw/2)
    c_yaw = np.cos(yaw/2)

    qx = s_roll * c_pitch * c_yaw - c_roll * s_pitch * s_yaw
    qy = c_roll * s_pitch * c_yaw + s_roll * c_pitch * s_yaw
    qz = c_roll * c_pitch * s_yaw - s_roll * s_pitch * c_yaw
    qw = c_roll * c_pitch * c_yaw + s_roll * s_pitch * s_yaw

    return [qx, qy, qz, qw]


def point_to_pose(point):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = point[0]
    p.pose.position.y = point[1]

    end_rotation = quaternion_from_euler(0, 0, point[2])

    p.pose.orientation.x = end_rotation[0]
    p.pose.orientation.y = end_rotation[1]
    p.pose.orientation.z = end_rotation[2]
    p.pose.orientation.w = end_rotation[3]

    return p


def apply_vector_to_frame(transform, vector):
    frame_rotation = euler_from_quaternion(transform.transform.rotation)[2]
    a, b = math.cos(frame_rotation), math.sin(frame_rotation)
    R = np.array([[a, b],
                  [-b, a]])

    vector = np.array(vector)
    d = R.dot(vector[0:2])

    return [transform.transform.translation.x + d[0],
            transform.transform.translation.y + d[1],
            -frame_rotation + vector[2]]


def point_reached(pose_transform, target):
    dist = math.sqrt((pose_transform.transform.translation.x - target.pose.position.x)
                     ** 2 + (pose_transform.transform.translation.y - target.pose.position.y) ** 2)
    pose_rotation = euler_from_quaternion(pose_transform.transform.rotation)[2]
    target_rotation = euler_from_quaternion(target.pose.orientation)[2]

    return dist <= DIST_TOLERANCE and normilize_angle(pose_rotation - target_rotation) <= ANGLE_TOLERANCE


class TaskNode(Node):

    def __init__(self):
        super().__init__('task_node')
        self.task_timer = self.create_timer(0.1, self.task_cycle)

        self.busy = False

        self.plan = {
            'init': {'action': self.check_init},
            'look_for_cube': {'action': self.check_cube},
            'reach_cube_1': {'action': self.reach_cube_1},
            'push_cube_1': {'action': self.push_cube_1},
            'go_to_zone_1': {'action': self.goto_zone_1},
            'go_to_zone_2_center': {'action': self.goto_zone_2_center},
            'reach_cube_2': {'action': self.reach_cube_2},
            'push_cube_2': {'action': self.push_cube_2},
        }

        for k, v in self.plan.items():
            self.plan[k]['completed'] = False

        self.pose_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('points', "{}")
        self.points = loads(self.get_parameter(
            'points').get_parameter_value().string_value)
        

    def task_cycle(self):
        if self.busy:
            return

        for k, v in self.plan.items():
            if not v['completed']:
                def obertochka():
                    self.get_logger().info('Starting step ' + k)
                    self.busy = True
                    v['action']()
                    self.busy = False
                    v['completed'] = True
                    self.get_logger().info('Completed step ' + k)

                thr = threading.Thread(target=obertochka)
                thr.start()

                break

    def get_transform(self, parent_frame, child_frame):
        while True:
            try:
                return self.tf_buffer.lookup_transform(
                    parent_frame, child_frame, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                self.get_logger().warn('waiting for transform from ' +
                                       parent_frame + ' to ' + child_frame)
            sleep(1)

    def check_init(self):
        self.get_transform("map", "base_link")

    def check_cube(self):
        self.get_transform("map", "cube")

    def go_to_point(self, point):
        pose = point_to_pose(point)
        self.pose_publisher.publish(pose)
        while True:
            self_pose_t = self.get_transform("map", "base_link")
            if point_reached(self_pose_t, pose):
                break
            sleep(0.1)
        sleep(3)

    def reach_cube_1(self):
        t = self.get_transform("map", "cube")
        self.go_to_point(apply_vector_to_frame(t, [0.35, 0.0, -math.pi]))

    def push_cube_1(self):
        t = self.get_transform("map", "base_link")
        self.go_to_point(apply_vector_to_frame(t, [0.3, 0.0, 0.0]))

        t = self.get_transform("map", "base_link")
        self.go_to_point(apply_vector_to_frame(t, [-0.2, 0.0, 0.0]))

    def goto_zone_1(self):
        zone_1_point = self.points["zone_1"]
        self.go_to_point([zone_1_point["x"], zone_1_point["y"], zone_1_point["yaw"]])

    def goto_zone_2_center(self):
        zone_2_1_point = self.points["zone_2_1"]
        self.go_to_point([zone_2_1_point["x"], zone_2_1_point["y"], zone_2_1_point["yaw"]])
        zone_2_center = self.points["zone_2_center"]
        self.go_to_point([zone_2_center["x"], zone_2_center["y"], zone_2_center["yaw"]])

    def reach_cube_2(self):
        t = self.get_transform("map", "cube")
        self.go_to_point(apply_vector_to_frame(t, [-0.2, 0.0, 0.0]))

    def push_cube_2(self):
        zone_2_center = self.points["zone_2_center"]
        self.go_to_point([zone_2_center["x"], zone_2_center["y"], normilize_angle(math.pi + zone_2_center["yaw"])])
        zone_2_to_push = self.points["zone_2_to_push"]
        self.go_to_point([zone_2_to_push["x"], zone_2_to_push["y"], zone_2_to_push["yaw"]])
        t = self.get_transform("map", "base_link")
        self.go_to_point(apply_vector_to_frame(t, [0.1, 0.0, 0.0]))
        self.go_to_point([zone_2_center["x"], zone_2_center["y"], zone_2_center["yaw"]])
        

def main(args=None):
    rclpy.init(args=args)
    task_node = TaskNode()

    rclpy.spin(task_node)

    task_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
