#!/usr/bin/env python
import os.path
import shlex
import subprocess

import psutil
import rospy
import tf

import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from air_hockey_msgs.msg import PlannerRequest

from air_hockey_puck_tracker.srv import GetPuckState


def print_(x, N=5):
    for i in range(N):
        print()
    print(x)
    for i in range(N):
        print()


class NeuralPlannerHittingTestNode:
    def __init__(self):
        rospy.init_node("neural_planner_hitting_test", anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.planner_request_publisher = rospy.Publisher("/neural_planner/plan_trajectory", PlannerRequest,
                                                         queue_size=5)
        self.replanner_request_publisher = rospy.Publisher("/neural_planner/replan_trajectory", PlannerRequest,
                                                           queue_size=5)
        self.robot_state_subscriber = rospy.Subscriber("/iiwa_front/joint_states", JointState, self.set_robot_state)
        self.robot_joint_pose = None
        self.robot_joint_velocity = None
        rospy.wait_for_service('/iiwa_front/get_puck_state')
        self.get_puck_state = rospy.ServiceProxy('/iiwa_front/get_puck_state', GetPuckState)
        rospy.sleep(2.)
        trans, _ = self.tf_listener.lookupTransform('/F_link_0', '/TableAway', rospy.Time(0))
        # trans = [2.4, 0., 0.1]
        self.goal = trans
        self.gazebo = rospy.get_param("/gazebo/time_step", "") != ""
        # self.gazebo = False

    def set_robot_state(self, msg):
        self.robot_joint_pose = msg.position[:7]
        self.robot_joint_velocity = msg.velocity[:7]

    def record_rosbag(self, time):
        path = os.path.dirname(os.path.dirname(__file__))
        command = "bash " + os.path.join(path, f"record_rosbag.sh {time}")
        command = shlex.split(command)
        rosbag_proc = subprocess.Popen(command)
        rospy.sleep(1.0)

    def hit(self):
        while not rospy.is_shutdown():
            inp = input("Press enter to hit or insert anything to leave")
            if inp:
                break
            self.randomize_puck_pose()
            self.record_rosbag(6)
            if self.request_hit_plan():
                pass
            rospy.sleep(5.0)

    def hit_with_bounce(self):
        while not rospy.is_shutdown():
            inp = input("Press enter to hit or insert anything to leave")
            if inp:
                break
            self.randomize_puck_pose()
            self.record_rosbag(6)
            if self.request_hit_plan(bounce=-1):
                pass
            rospy.sleep(5.0)

    def fake_hit_and_replan(self):
        while not rospy.is_shutdown():
            inp = input("Press enter to hit or insert anything to leave")
            if inp:
                break
            # fake hit
            if self.request_hit_plan((1.1, 0.3, 0.0)):
                # for i in range(10):
                rospy.sleep(0.15)
                self.request_hit_replan()

    def lissajoux_hit(self):
        while not rospy.is_shutdown():
            inp = input("Press enter to hit or insert anything to leave")
            if inp:
                break
            self.record_rosbag(15)
            rospy.sleep(1.0)
            # lissajoux
            pr = PlannerRequest()
            pr.q_0 = self.robot_joint_pose
            pr.q_dot_0 = self.robot_joint_velocity
            pr.q_ddot_0 = np.zeros(7)
            pr.tactic = 2
            pr.header.stamp = rospy.Time.now()
            self.planner_request_publisher.publish(pr)
            rospy.sleep(3.0)
            self.request_hit_replan()

    def read_puck_pose(self, t):
        dx = []
        dy = []
        x = []
        y = []
        for i in range(10):
            resp = self.get_puck_state(t)
            #x.append(resp.prediction.x)
            #y.append(resp.prediction.y)
            dx.append(resp.prediction.dx)
            dy.append(resp.prediction.dy)
        trans, _ = self.tf_listener.lookupTransform('/F_link_0', resp.prediction.header.frame_id, rospy.Time(0))
        x = resp.prediction.x + trans[0]
        y = resp.prediction.y + trans[1]
        #x = np.mean(x) + trans[0]
        #y = np.mean(y) + trans[1]
        dx = np.mean(dx)#resp.prediction.dx
        dy = np.mean(dy)#resp.prediction.dy
        th = np.arctan2(dy, dx)
        v = np.sqrt(dy**2 + dx**2)
        return x, y, th, v, resp

    def demo_bounce_and_hit(self):
        xlow = 0.9
        xhigh = 1.15
        xy_valid = lambda x, y: xlow < x < xhigh and -0.35 < y < 0.35
        expected_time = 0.6
        while not rospy.is_shutdown():
            inp = input("Press enter to enable moving")
            if inp:
                break
            self.record_rosbag(10)
            rospy.sleep(1.0)
            self.set_puck_pose(-0.5, 0., 0., 0.)
            x, y, th, v, resp = self.read_puck_pose(expected_time)
            #self.request_plan((x, y, np.pi/2), expected_velocity=0.2)
            self.request_move_plan(x - 0.1, np.maximum(y - 0.3, -0.4)) # if not working change to -0.4
            rospy.sleep(0.8)
            #rospy.sleep(0.35)
            #self.request_hit_replan((x, y, np.pi / 2), expected_velocity=0.5)
            self.request_hit_plan((x, y, np.pi / 2 - 0.1), expected_velocity=1.6)
            rospy.sleep(1.5)
            for i in range(100):
                print(i)
                expected_time = 0.7
                x, y, th, v, resp = self.read_puck_pose(expected_time)
                print(x, y)
                if xy_valid(x, y):
                    print("TRY TO HIT", x, y)
                    #self.request_hit_plan((x, y), expected_time + 0.035)
                    self.request_hit_plan((x, y), expected_time + 0.055)
                    rospy.sleep(3.0)
                    break

            #self.request_move_plan(0.65, 0.)

            #break
            #x, y, resp = self.read_puck_pose(expected_time)
            #print(x)
            #if xy_valid(x, y):
            #    print("TRY TO HIT", x, y)
            #    self.request_plan((x, y), expected_time)
    def demo_straight_hitting(self):
        xlow = 0.8
        xhigh = 1.15
        xy_valid = lambda x, y: xlow < x < xhigh and -0.35 < y < 0.35
        expected_time = 0.6
        while not rospy.is_shutdown():
            inp = input("Press enter to enable moving")
            if inp:
                break
            #self.record_rosbag(10)
            #rospy.sleep(1.0)
            x, y, th, v, resp = self.read_puck_pose(expected_time)
            x_init = x - 0.05
            y_init = np.maximum(y - 0.3, -0.4)
            self.request_move_plan(x_init, y_init)
            rospy.sleep(1.0)
            #rospy.sleep(0.35)
            #self.request_hit_replan((x, y, np.pi / 2), expected_velocity=0.5)
            r = 0.06
            th = np.pi/2
            xh = x - r * np.cos(th)
            yh = y - r * np.sin(th)
            self.request_hit_plan((xh, yh, th), expected_velocity=1.1)
            rospy.sleep(1.5)
            for i in range(100):
                print(i)
                x, y, th, v, resp = self.read_puck_pose(expected_time)
                th = np.pi / 2
                xh = x - r * np.cos(th)
                yh = y - r * np.sin(th)
                print(x, y)
                if xy_valid(xh, yh):
                    print("TRY TO HIT", xh, yh)
                    self.request_hit_plan((xh, yh, th), expected_time + 0.0)
                    rospy.sleep(3.0)
                    break
            rospy.sleep(2.5)

            self.request_move_plan(0.65, 0.)

    def demo_bounce_hitting(self):
        expected_time = 0.7
        while not rospy.is_shutdown():
            inp = input("Press enter to enable moving")
            if inp:
                break
            x, y, th, v, resp = self.read_puck_pose(expected_time)
            self.request_move_plan(1.2, -0.3)
            rospy.sleep(1.5)
            #self.request_move_plan(1.1, -0.4)
            #rospy.sleep(1.0)

            #rospy.sleep(0.35)
            #self.request_hit_replan((x, y, np.pi / 2), expected_velocity=0.5)
            r = 0.06
            th = np.pi - 0.2
            xh = x - r * np.cos(th)
            yh = y - r * np.sin(th)
            self.request_hit_plan((xh, yh, th))
            rospy.sleep(2.5)
            self.request_move_plan(0.65, 0.)

    def demo_slice_hitting(self):
        expected_time = 0.7
        while not rospy.is_shutdown():
            inp = input("Press enter to enable moving")
            if inp:
                break
            x, y, th, v, resp = self.read_puck_pose(expected_time)
            rospy.sleep(1.5)
            r = 0.06
            th = np.pi/4
            xh = x - r * np.sin(th)
            yh = y + r * np.cos(th)
            self.request_hit_plan((xh, yh, th))

    def moving_puck_hitting_test(self):
        xlow = 0.9
        xhigh = 1.1
        xy_valid = lambda x, y: xlow < x < xhigh and -0.35 < y < 0.35
        allow_planning = False
        while not rospy.is_shutdown():
            if not allow_planning:
                inp = input("Press enter to enable moving")
                if inp:
                    break
                else:
                    self.record_rosbag(15)
                    allow_planning = True
            self.randomize_puck_pose()
            expected_time = 0.8
            x, y, th, v, resp = self.read_puck_pose(expected_time)
            if xy_valid(x, y):
                print("TRY TO HIT", x, y)
                self.request_hit_plan((x, y), expected_time)
                # delay = 0.3
                # rospy.sleep(delay)
                # new_expected_time = expected_time - delay - 0.05
                # x, y, resp = self.read_puck_pose(new_expected_time)
                # print(f"TRY TO HIT AFTER {delay}s: ", x, y)
                # if xy_valid(x, y):
                #    print("VALID")
                #    self.request_replan((x, y), new_expected_time)

                allow_planning = False

    def moving_puck_defending_test(self):
        xlow = 0.85
        xhigh = 1.0
        xy_valid = lambda x, y: xlow < x < xhigh and -0.35 < y < 0.35
        allow_planning = False
        while not rospy.is_shutdown():
            if not allow_planning:
                inp = input("Press enter to enable moving")
                if inp:
                    break
                else:
                    #self.request_move_plan(0.65, 0.)
                    #self.record_rosbag(15)
                    allow_planning = True
            self.randomize_puck_pose()
            expected_time = 0.6
            x, y, th, v, resp = self.read_puck_pose(expected_time)
            r = 0.07975
            beta = 0.
            gamma = (beta - np.pi + th) / 2.
            px = x + r * np.cos(gamma)
            py = y + r * np.sin(gamma)
            #print("PUCK V ANGLE: ", th)
            #print("COMPUTED APPROACHING ANGLE: ", gamma)
            if xy_valid(px, py):
                #print("PUCK EXPECTED AT: ", x, y)
                #print("TRY TO HIT AT: ", px, py)
                self.request_move_plan(px, py, expected_time - 0.1)
                rospy.sleep(2.)
                self.request_move_plan(0.65, 0.)
                # delay = 0.3
                # rospy.sleep(delay)
                # new_expected_time = expected_time - delay - 0.05
                # x, y, resp = self.read_puck_pose(new_expected_time)
                # print(f"TRY TO HIT AFTER {delay}s: ", x, y)
                # if xy_valid(x, y):
                #    print("VALID")
                #    self.request_replan((x, y), new_expected_time)

                allow_planning = False

    def randomize_puck_pose(self):
        x = -0.5 + 0.2 * np.random.rand()
        y = -0.4 + 0.8 * np.random.rand()
        v = 0.0
        a = np.pi * (2 * np.random.random() - 1.)
        vx = v * np.cos(a)
        vy = v * np.sin(a)
        self.set_puck_pose(x, y, vx, vy)

    def set_puck_pose(self, x, y, vx, vy):
        if self.gazebo:
            state_msg = ModelState()
            state_msg.model_name = 'puck'
            state_msg.pose.position.x = x
            state_msg.pose.position.y = y
            state_msg.twist.linear.x = vx
            state_msg.twist.linear.y = vy
            state_msg.pose.position.z = 0.0
            state_msg.pose.orientation.x = 0
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0
            state_msg.reference_frame = "air_hockey_table::Table"
            rospy.wait_for_service('/gazebo/set_model_state')
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(state_msg)
            rospy.sleep(0.5)

    def prepare_move_planner_request(self, x, y, expected_time):
        pr = PlannerRequest()
        pr.q_0 = self.robot_joint_pose
        pr.q_dot_0 = self.robot_joint_velocity
        pr.q_ddot_0 = [0.] * 7
        end_point = Point(float(x), float(y), float(0.16))
        pr.end_point = end_point
        pr.expected_time = expected_time
        pr.tactic = 1
        pr.header.stamp = rospy.Time.now()
        return pr


    def prepare_hit_planner_request(self, task=None, expected_time=-1., expected_velocity=-1., bounce=0):
        print("PREPARE PLANNER REQUEST")

        def get_desired_xyth(puck_pose, goal_pose, mallet_pose, bounce=0):
            puck_pose = list(puck_pose)
            goal_pose = list(goal_pose)
            mallet_pose = list(mallet_pose)

            # dy = 0.11
            dy = 0.0
            print("GOAL POSE Y BEFORE: ", goal_pose[1])
            if puck_pose[1] > 0.125:
                goal_pose[1] += dy
            elif puck_pose[1] < -0.125:
                goal_pose[1] -= dy
            print("GOAL POSE Y UPDATED: ", goal_pose[1])
            puck_goal_x = goal_pose[0] - puck_pose[0]
            puck_goal_y = goal_pose[1] - puck_pose[1]
            print(puck_goal_y, puck_goal_x)
            th = np.arctan2(puck_goal_y, puck_goal_x)
            w = 1.038
            r = 0.03165
            xg = goal_pose[0]
            yb = w/2 - r
            x = puck_pose[0]
            y = puck_pose[1]
            # Upper band
            if bounce == -1:
                xbu = (yb * x - xg * y + xg * yb) / (2 * yb - y)
                thu = np.arctan2(yb - y, xbu - x)
                upper_valid = xg > xbu and xbu > x
                if upper_valid:
                    th = thu
            # Lower band
            elif bounce == 1:
                xbl = (yb * x + xg * y + xg * yb) / (2 * yb + y)
                thl = np.arctan2(-y - yb, xbl - x)
                lower_valid = xg > xbl and xbl > x
                if lower_valid:
                    th = thl
            print("TH", th)
            # r = 0.07975
            #r = 0.05
            r = 0.06
            x = puck_pose[0] - r * np.cos(th)
            y = puck_pose[1] - r * np.sin(th)
            return x, y, th

        try:
            trans, rot = self.tf_listener.lookupTransform('/F_link_0', '/Puck', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        # trans = [0.9585, -0.39, 0.16]
        mallet_pose, _ = self.tf_listener.lookupTransform('/F_link_0', '/F_striker_tip', rospy.Time(0))
        if task is not None:
            if len(task) == 3:
                x, y, th = task
            elif len(task) == 2:
                x, y, th = get_desired_xyth(task, self.goal, mallet_pose)
        else:
            x, y, th = get_desired_xyth(trans, self.goal, mallet_pose, bounce)

        print("DESIRED POSE:", x, " ", y)
        if not self.gazebo:
            if x < 0.68 or x > 1.3:
                print("Puck not in x range")
                return None
            if y < -0.41 or y > 0.41:
                print("Puck not in y range")
                return None
        # y *= 0.93 + 0.07 * (x - 0.7) / (1.3 - 0.7)
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
        pr.expected_time = expected_time
        pr.expected_velocity = expected_velocity
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

    def request_hit_plan(self, task=None, expected_time=-1., expected_velocity=-1., bounce=0):
        pr = self.prepare_hit_planner_request(task, expected_time, expected_velocity, bounce)
        if pr is None:
            return False
        self.planner_request_publisher.publish(pr)
        return True

    def request_hit_replan(self, task=None, expected_time=-1., expected_velocity=-1.):
        pr = self.prepare_hit_planner_request(task, expected_time, expected_velocity)
        if pr is None:
            return False
        self.replanner_request_publisher.publish(pr)
        return True

    def request_move_plan(self, x, y, expected_time=-1.):
        pr = self.prepare_move_planner_request(x, y, expected_time)
        if pr is None:
            return False
        self.planner_request_publisher.publish(pr)
        return True

    def request_move_replan(self, x, y, expected_time=-1.):
        pr = self.prepare_move_planner_request(x, y, expected_time)
        if pr is None:
            return False
        self.replanner_request_publisher.publish(pr)
        return True


if __name__ == '__main__':
    node = NeuralPlannerHittingTestNode()
    # node.hit_xyth(0.85, -0.1891, -1.0105)
    # node.hit_xyth(0.8586, -0.2136, -1.12)
    # node.hit_xyth(0.9, -0.3, -0.87606)
    # node.hit_xyth(1.0, -0.3, 0.1)
    node.hit()
    #node.lissajoux_hit()
    #node.hit_with_bounce()
    #node.moving_puck_hitting_test()
    # node.moving_puck_defending_test()
    #node.demo_bounce_and_hit()
    #node.demo_straight_hitting()
    #node.demo_bounce_hitting()
    # node.demo_slice_hitting()

    # node.fake_hit_and_replan()
