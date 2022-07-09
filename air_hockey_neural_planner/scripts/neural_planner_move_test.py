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


class NeuralPlannerMoveTestNode:
    def __init__(self):
        rospy.init_node("neural_planner_move_test", anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.planner_request_publisher = rospy.Publisher("/neural_planner/plan_trajectory", PlannerRequest,
                                                         queue_size=5)
        self.replanner_request_publisher = rospy.Publisher("/neural_planner/replan_trajectory", PlannerRequest,
                                                         queue_size=5)
        self.robot_state_subscriber = rospy.Subscriber("/iiwa_front/joint_states", JointState, self.set_robot_state)
        self.robot_joint_pose = None
        self.robot_joint_velocity = None
        rospy.sleep(2.)

    def set_robot_state(self, msg):
        self.robot_joint_pose = msg.position[:7]
        self.robot_joint_velocity = msg.velocity[:7]

    def move(self):
        while not rospy.is_shutdown():
            inp = input("Press enter to move or insert anything to leave")
            if inp:
                break
            print("ROS TIME 1:", rospy.Time.now().to_sec())
            self.request_random_plan()
            rospy.sleep(0.1)
            print("ROS TIME 2:", rospy.Time.now().to_sec())
            self.request_random_replan()

    def prepare_random_plan_request(self):
        pr = PlannerRequest()
        pr.q_0 = self.robot_joint_pose
        pr.q_dot_0 = self.robot_joint_velocity
        pr.q_ddot_0 = np.zeros(7)

        x = 0.6 * np.random.rand() + 0.6
        y = 0.8 * np.random.rand() - 0.4
        end_point = Point(x, y, 0.16)
        pr.end_point = end_point
        pr.tactic = 1
        pr.header.stamp = rospy.Time.now()
        return pr

    def request_random_plan(self):
        pr = self.prepare_random_plan_request()
        self.planner_request_publisher.publish(pr)

    def request_random_replan(self):
        pr = self.prepare_random_plan_request()
        self.replanner_request_publisher.publish(pr)


if __name__ == '__main__':
    node = NeuralPlannerMoveTestNode()
    node.move()
