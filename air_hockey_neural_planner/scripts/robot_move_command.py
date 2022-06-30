#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from air_hockey_referee.msg import GameStatus

import numpy as np


class RobotMoveCommand:
    def __init__(self):
        rospy.init_node("robot_move_command", anonymous=True)
        self.iiwa_front_publisher = rospy.Publisher("/iiwa_front/bspline_ff_joint_trajectory_controller/command", JointTrajectory,
                                                    queue_size=5)
        #self.iiwa_back_publisher = rospy.Publisher(f"/iiwa_back/{back_controller_type}/command", JointTrajectory,
        #                                           queue_size=5)

        self.iiwa_front_trajectory = JointTrajectory()
        for i in range(7):
            self.iiwa_front_trajectory.joint_names.append(f"F_joint_{i + 1}")
        #self.iiwa_back_trajectory = JointTrajectory()
        #for i in range(7):
        #    self.iiwa_back_trajectory.joint_names.append(f"B_joint_{i + 1}")

    def move(self, q):
            traj_point_goal = JointTrajectoryPoint()
            traj_point_goal.positions = np.zeros(7)
            traj_point_goal.velocities = np.zeros(7)
            traj_point_goal.accelerations = np.zeros(7)

            traj_point_goal.positions = q
            traj_point_goal.time_from_start = rospy.Time(3.0)
            self.iiwa_front_trajectory.points = [traj_point_goal]
            self.iiwa_front_trajectory.header.stamp = rospy.Time.now()
            #self.iiwa_back_trajectory.points = [traj_point_goal]
            #self.iiwa_back_trajectory.header.stamp = rospy.Time.now()

            self.iiwa_front_publisher.publish(self.iiwa_front_trajectory)
            #self.iiwa_back_publisher.publish(self.iiwa_back_trajectory)


if __name__ == '__main__':
    node = RobotMoveCommand()
    rospy.sleep(2.)
    node.move([0., 0.5, 0., -0.5, 0., 1.92, 0.])

