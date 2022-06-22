#!/usr/bin/env python
import rospy
import tf

import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from air_hockey_neural_planner.msg import PlannerRequest


def print_(x, N=5):
    for i in range(N):
        print()
    print(x)
    for i in range(N):
        print()


class NeuralPlannerTestNode:
    def __init__(self):
        rospy.init_node("neural_planner_node_test", anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.planner_request_publisher = rospy.Publisher("/neural_planner/plan_trajectory", PlannerRequest,
                                                         queue_size=5)
        self.robot_state_subscriber = rospy.Subscriber("/iiwa_front/joint_states", JointState, self.set_robot_state)
        self.robot_joint_pose = None
        self.robot_joint_velocity = None
        rospy.sleep(2.)
        trans, _ = self.tf_listener.lookupTransform('/F_link_0', '/TableAway', rospy.Time(0))
        self.goal = trans

    def set_robot_state(self, msg):
        self.robot_joint_pose = msg.position[:7]
        self.robot_joint_velocity = msg.velocity[:7]

    def request_plan(self):
        def get_desired_xyth(puck_pos, goal_pose):
            puck_goal_x = goal_pose[0] - puck_pos[0]
            puck_goal_y = goal_pose[1] - puck_pos[1]
            print(puck_goal_y, puck_goal_x)
            th = np.arctan2(puck_goal_y, puck_goal_x)
            print(th)
            #r = 0.07
            r = 0.1
            x = puck_pos[0] - r * np.cos(th)
            y = puck_pos[1] - r * np.sin(th)
            return x, y, th

        state_msg = ModelState()
        state_msg.model_name = 'puck'
        state_msg.pose.position.x = -0.5
        state_msg.pose.position.y = -0.2
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
        while not rospy.is_shutdown():
            try:
                trans, rot = self.tf_listener.lookupTransform('/F_link_0', '/Puck', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            print_(trans)
            x, y, th = get_desired_xyth(trans, self.goal)
            pr = PlannerRequest()
            pr.q_0 = self.robot_joint_pose
            print(pr.q_0)
            print(type(pr.q_0))
            pr.q_dot_0 = self.robot_joint_velocity
            pr.q_ddot_0 = np.zeros(7)
            end_point = Point(x, y, 0.16)
            pr.end_point = end_point
            pr.hitting_angle = th
            self.planner_request_publisher.publish(pr)
            break


if __name__ == '__main__':
    node = NeuralPlannerTestNode()
    node.request_plan()
