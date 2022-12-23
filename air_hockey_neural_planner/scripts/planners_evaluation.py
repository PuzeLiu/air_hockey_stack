#!/usr/bin/env python
import os.path
import shlex
import signal
import subprocess

import psutil
import rospy
import tf

import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from air_hockey_msgs.msg import PlannerRequest, PlannerStatus

from air_hockey_puck_tracker.srv import GetPuckState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def print_(x, N=5):
    for i in range(N):
        print()
    print(x)
    for i in range(N):
        print()


class PlannersEvaluationNode:
    def __init__(self):
        rospy.init_node("planners_evaluation", anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.planner_request_publisher = rospy.Publisher("/neural_planner/plan_trajectory", PlannerRequest,
                                                         queue_size=5)
        self.robot_state_subscriber = rospy.Subscriber("/iiwa_front/joint_states", JointState, self.set_robot_state)
        self.planner_status_publisher = rospy.Subscriber(f"/neural_planner/status", PlannerStatus,
                                                         self.kill_rosbag_proc)
        self.iiwa_front_publisher = rospy.Publisher(f"/iiwa_front/bspline_ff_joint_trajectory_controller/command",
                                                    JointTrajectory,
                                                    queue_size=5)
        self.robot_joint_pose = None
        self.robot_joint_velocity = None
        self.q_base = rospy.get_param("/air_hockey/agent/q_ref")
        rospy.wait_for_service('/iiwa_front/get_puck_state')
        self.get_puck_state = rospy.ServiceProxy('/iiwa_front/get_puck_state', GetPuckState)
        rospy.sleep(2.)
        trans, _ = self.tf_listener.lookupTransform('/F_link_0', '/TableAway', rospy.Time(0))
        self.table, _ = self.tf_listener.lookupTransform('/F_link_0', '/Table', rospy.Time(0))
        self.goal = trans
        self.rosbag_proc = None
        self.is_moving = False
        # self.method = "sst"
        #self.method = "mpcmpnet"
        self.method = "mpcmpnet_newp"
        #self.method = "ours"
        # self.method = "iros"
        # self.method = "nlopt"
        # self.method = "cbirrt"
        #self.method = "cbirrt_vel"

    def move_to_base(self):
        iiwa_front_msg = JointTrajectory()
        pt = JointTrajectoryPoint()
        pt.positions = self.q_base
        pt.velocities = [0.] * 7
        pt.accelerations = [0.] * 7
        pt.time_from_start = rospy.Duration(2.0)
        iiwa_front_msg.points.append(pt)
        iiwa_front_msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        iiwa_front_msg.joint_names = [f"F_joint_{i}" for i in range(1, 8)]
        self.iiwa_front_publisher.publish(iiwa_front_msg)

    def kill_rosbag_proc(self, msg):
        print("XDDDDD")
        rospy.sleep(max(5., msg.planned_hitting_time + 2.))
        # rospy.sleep(max(4., msg.planned_hitting_time + 1.))
        if self.rosbag_proc is not None:
            os.kill(self.rosbag_proc.pid, signal.SIGTERM)
        rospy.sleep(1.)
        self.rosbag_proc = None
        self.is_moving = False

    def set_robot_state(self, msg):
        self.robot_joint_pose = msg.position[:7]
        self.robot_joint_velocity = msg.velocity[:7]

    def record_rosbag(self, x, y):
        # path = os.path.dirname(os.path.dirname(__file__))
        # command = "bash " + os.path.join(path, f"record_rosbag.sh {time}")
        name = f"data/hitting_exp/{self.method}/x{x:.2f}_y{y:.2f}.bag"
        command = "rosbag record " \
                  f"-O {name} " \
                  "/iiwa_front/bspline_ff_joint_trajectory_controller/state " \
                  "/iiwa_front/joint_states /tf " \
                  "/neural_planner/status /neural_planner/plan_trajectory"
        command = shlex.split(command)
        self.rosbag_proc = subprocess.Popen(command)
        rospy.sleep(1.0)

    def evaluate(self):
        xs = np.linspace(0.8, 1.2, 9)
        # xs = np.linspace(1.15, 1.2, 2)
        # xs = np.linspace(1.05, 1.2, 4)
        ys = np.linspace(-0.4, 0.4, 9)
        # xs = np.linspace(0.8, 1.2, 3)
        # ys = np.linspace(-0.4, 0.4, 3)
        # self.move_to_base()
        # rospy.sleep(3.0)
        # xs = [1.]
        # ys = [0.2]
        # xs = [1.15]
        # ys = [0.1]
        for i, x in enumerate(xs):
           for j, y in enumerate(ys):
               self.set_puck_pose(x, y, 0., 0.)
               self.record_rosbag(x, y)
               self.request_hit_plan(x, y)
               self.is_moving = True
               i = 0
               while self.is_moving:
                   print("XD", self.is_moving, i)
                   rospy.sleep(0.1)
                   i += 1
                   pass
               self.move_to_base()
               rospy.sleep(4.)

    def for_video(self):
        xs = [0.8, 0.95, 1., 0.9, 1.15]
        ys = [-0.4, 0.2, 0.0, -0.1, 0.3]
        for x, y in zip(xs, ys):
            self.set_puck_pose(x, y, 0., 0.)
            self.request_hit_plan(x, y)
            self.is_moving = True
            i = 0
            while self.is_moving:
                print("XD", self.is_moving, i)
                rospy.sleep(0.1)
                i += 1
                pass
            self.move_to_base()
            rospy.sleep(4.)

    def set_puck_pose(self, x, y, vx, vy):
        state_msg = ModelState()
        state_msg.model_name = 'puck'
        state_msg.pose.position.x = x - self.table[0]
        state_msg.pose.position.y = y - self.table[1]
        state_msg.twist.linear.x = vx
        state_msg.twist.linear.y = vy
        state_msg.pose.position.z = 0.
        state_msg.pose.orientation.x = 0.
        state_msg.pose.orientation.y = 0.
        state_msg.pose.orientation.z = 0.
        state_msg.pose.orientation.w = 0.
        state_msg.reference_frame = "air_hockey_table::Table"
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(state_msg)
        rospy.sleep(0.5)

    def prepare_hit_planner_request(self, x, y):
        print("PREPARE PLANNER REQUEST")

        def get_desired_xyth(puck_pose, goal_pose):
            puck_pose = list(puck_pose)
            goal_pose = list(goal_pose)

            puck_goal_x = goal_pose[0] - puck_pose[0]
            puck_goal_y = goal_pose[1] - puck_pose[1]
            print(puck_goal_y, puck_goal_x)
            th = np.arctan2(puck_goal_y, puck_goal_x)
            # r = 0.07975
            r = 0.07
            # r = 0.06
            # r = 0.10
            x = puck_pose[0] - r * np.cos(th)
            y = puck_pose[1] - r * np.sin(th)
            return x, y, th

        x, y, th = get_desired_xyth((x, y), self.goal)

        print("DESIRED POSE:", x, " ", y)
        pr = PlannerRequest()
        pr.q_0 = self.robot_joint_pose
        print(pr.q_0)
        print(type(pr.q_0))
        pr.q_dot_0 = self.robot_joint_velocity
        pr.q_ddot_0 = np.zeros(7)
        hit_point = Point(x, y, 0.16)
        pr.hit_point = hit_point
        pr.hit_angle = th
        print("TH:", th)
        pr.expected_time = -1.
        pr.expected_velocity = -1.
        pr.tactic = 0
        pr.header.stamp = rospy.Time.now()
        return pr

    def hit_xyth(self, x, y, th):
        pr = PlannerRequest()
        pr.q_0 = self.robot_joint_pose
        pr.q_dot_0 = self.robot_joint_velocity
        pr.q_ddot_0 = np.zeros(7)
        hit_point = Point(x, y, 0.16)
        pr.hit_point = hit_point
        pr.hit_angle = th
        pr.tactic = 0
        pr.header.stamp = rospy.Time.now()
        self.planner_request_publisher.publish(pr)
        return True

    def request_hit_plan(self, x, y):
        pr = self.prepare_hit_planner_request(x, y)
        if pr is None:
            return False
        self.planner_request_publisher.publish(pr)
        return True


if __name__ == '__main__':
    node = PlannersEvaluationNode()
    #node.for_video()
    node.evaluate()
