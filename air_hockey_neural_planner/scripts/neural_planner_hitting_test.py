#!/usr/bin/env python
import rospy
import tf

import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from air_hockey_neural_planner.msg import PlannerRequest

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

    def hit(self):
        while not rospy.is_shutdown():
            inp = input("Press enter to hit or insert anything to leave")
            if inp:
                break
            if self.request_plan():
                # for i in range(10):
                #rospy.sleep(0.1)
                #print("ROS TIME:", rospy.Time.now().to_sec())
                #self.request_replan()
                pass

    def fake_hit_and_replan(self):
        while not rospy.is_shutdown():
            inp = input("Press enter to hit or insert anything to leave")
            if inp:
                break
            # fake hit
            if self.request_plan((1.1, 0.3, 0.0)):
                # for i in range(10):
                rospy.sleep(0.15)
                self.request_replan()
                pass

    def read_puck_pose(self, t):
        resp = self.get_puck_state(t)
        trans, _ = self.tf_listener.lookupTransform('/F_link_0', resp.prediction.header.frame_id, rospy.Time(0))
        x = resp.prediction.x + trans[0]
        y = resp.prediction.y + trans[1]
        return x, y, resp

    def moving_puck_hitting_test(self):
        xlow = 0.9
        xhigh = 1.2
        xy_valid = lambda x, y: xlow < x < xhigh and -0.35 < y < 0.35
        allow_planning = False
        while not rospy.is_shutdown():
            if not allow_planning:
                inp = input("Press enter to enable moving")
                if inp:
                    break
                else:
                    allow_planning = True
            expected_time = 0.6
            x, y, resp = self.read_puck_pose(expected_time)
            print(x)
            if xy_valid(x, y):
                print("TRY TO HIT", x, y)
                self.request_plan((x, y), expected_time)
                #delay = 0.3
                #rospy.sleep(delay)
                #new_expected_time = expected_time - delay - 0.05
                #x, y, resp = self.read_puck_pose(new_expected_time)
                #print(f"TRY TO HIT AFTER {delay}s: ", x, y)
                #if xy_valid(x, y):
                #    print("VALID")
                #    self.request_replan((x, y), new_expected_time)


                allow_planning = False

    def prepare_planner_request(self, task=None, expected_time=-1., randomize_puck_pose=True):
        print("PREPARE PLANNER REQUEST")

        def get_desired_xyth(puck_pos, goal_pose, mallet_pose):
            #abs_puck_y = np.abs(puck_pos[1])
            #d = np.abs(puck_pos[0] - 0.9)
            #if abs_puck_y > 0.125:
            #    goal_pose[1] = (1 + 3.0 * np.exp(-100 * d ** 2)) * 0.125 * (abs_puck_y - 0.125) / (
            #                0.4 - 0.125) * np.sign(puck_pos[1])
                # goal_pose[1] = (1 + 3.0 * np.exp(-100 * d ** 2)) * 0.125 * (abs_puck_y - 0.125) / (0.4 - 0.125) * np.sign(puck_pos[1])
            print("GOAL POSE Y: ", goal_pose[1])
            puck_goal_x = goal_pose[0] - puck_pos[0]
            puck_goal_y = goal_pose[1] - puck_pos[1]
            print(puck_goal_y, puck_goal_x)
            th = np.arctan2(puck_goal_y, puck_goal_x)
            mallet_puck_x = puck_pos[0] - mallet_pose[0]
            mallet_puck_y = puck_pos[1] - mallet_pose[1]
            print(mallet_puck_y, mallet_puck_x)
            beta = np.arctan2(mallet_puck_y, mallet_puck_x)
            print("BETA", beta)
            alpha = np.abs(th - beta)
            print("ALPHA", alpha)
            # th *= (1 - (alpha / (np.pi / 2.))**(1.))
            print("TH", th)
            r = 0.07
            # r = 0.1
            x = puck_pos[0] - r * np.cos(th)
            y = puck_pos[1] - r * np.sin(th)
            return x, y, th

        if self.gazebo and randomize_puck_pose:
            state_msg = ModelState()
            state_msg.model_name = 'puck'
            state_msg.pose.position.x = -0.5 + 0.2 * np.random.rand()
            state_msg.pose.position.y = -0.4 + 0.8 * np.random.rand()
            v = 0.1
            state_msg.twist.linear.x = v * (2 * np.random.rand() - 1.)
            state_msg.twist.linear.y = v * (2 * np.random.rand() - 1.)
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
        try:
            trans, rot = self.tf_listener.lookupTransform('/F_link_0', '/Puck', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        # trans = [0.9585, -0.39, 0.16]
        mallet_pose, _ = self.tf_listener.lookupTransform('/F_link_0', '/F_striker_tip', rospy.Time(0))
        x, y, th = get_desired_xyth(trans, self.goal, mallet_pose)
        if task is not None:
            if len(task) == 3:
                x, y, th = task
            elif len(task) == 2:
                x, y, th = get_desired_xyth(task, self.goal, mallet_pose)

        print("DESIRED POSE:", x, " ", y)
        if not self.gazebo:
            if x < 0.7 or x > 1.3:
                print("Puck not in x range")
                return None
            if y < -0.4 or y > 0.4:
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
        pr.expected_time = expected_time
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



    def request_plan(self, task=None, expected_time=-1.):
        pr = self.prepare_planner_request(task, expected_time)
        if pr is None:
            return False
        self.planner_request_publisher.publish(pr)
        return True

    def request_replan(self, task=None, expected_time=-1.):
        pr = self.prepare_planner_request(task, expected_time, randomize_puck_pose=False)
        if pr is None:
            return False
        self.replanner_request_publisher.publish(pr)
        return True


if __name__ == '__main__':
    node = NeuralPlannerHittingTestNode()
    #node.hit_xyth(0.85, -0.1891, -1.0105)
    #node.hit_xyth(0.8586, -0.2136, -1.12)
    # node.hit_xyth(0.9, -0.3, -0.87606)
    #node.hit_xyth(1.0, -0.3, 0.1)
    node.hit()
    #node.fake_hit_and_replan()
    #node.moving_puck_hitting_test()
