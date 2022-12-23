#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from air_hockey_msgs.msg import GameStatus

import numpy as np


class GameState:
    STOP = 0
    PAUSE = 1
    START = 2


class RobotInitializerNode:
    def __init__(self):
        rospy.init_node("robot_initializer_node", anonymous=True)
        front_controller_type = rospy.get_param("~front_controllers")
        back_controller_type = rospy.get_param("~back_controllers")
        self.reference_configuration = rospy.get_param("/air_hockey/agent/q_ref")
        self.game_status_subscriber = rospy.Subscriber("/air_hockey_referee/game_status", GameStatus,
                                                       self.prepare_robots)
        self.iiwa_front_publisher = rospy.Publisher(f"/iiwa_front/{front_controller_type}/command", JointTrajectory,
                                                    queue_size=5)
        self.iiwa_back_publisher = rospy.Publisher(f"/iiwa_back/{back_controller_type}/command", JointTrajectory,
                                                   queue_size=5)

        self.iiwa_front_trajectory = JointTrajectory()
        for i in range(7):
            self.iiwa_front_trajectory.joint_names.append(f"F_joint_{i + 1}")
        self.iiwa_back_trajectory = JointTrajectory()
        for i in range(7):
            self.iiwa_back_trajectory.joint_names.append(f"B_joint_{i + 1}")

    def prepare_robots(self, msg):
        if msg.status == GameState.START or msg.status == GameState.STOP:
            traj_point_goal = JointTrajectoryPoint()
            traj_point_goal.positions = np.zeros(7)
            traj_point_goal.velocities = np.zeros(7)
            traj_point_goal.accelerations = np.zeros(7)

            traj_point_goal.positions = self.reference_configuration
            traj_point_goal.time_from_start = rospy.Time(3.0)
            self.iiwa_front_trajectory.points = [traj_point_goal]
            self.iiwa_front_trajectory.header.stamp = rospy.Time.now()
            self.iiwa_back_trajectory.points = [traj_point_goal]
            self.iiwa_back_trajectory.header.stamp = rospy.Time.now()

            self.iiwa_front_publisher.publish(self.iiwa_front_trajectory)
            #self.iiwa_back_publisher.publish(self.iiwa_back_trajectory)


if __name__ == '__main__':
    node = RobotInitializerNode()
    rospy.spin()

