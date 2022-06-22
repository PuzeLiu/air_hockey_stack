#!/usr/bin/env python3
import os.path
import numpy as np
import time
from copy import deepcopy
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == '__main__':
    # type_name = 'joint_feedforward'
    type_name = 'adrc'
    use_front = True

    topic_name = '/iiwa_front/' if use_front else '/iiwa_back/'
    joint_prefix = 'F' if use_front else 'B'

    print("pre init node")

    rospy.init_node("publish_traj_command", anonymous=True)
    print("post init node")
    cmdPub = rospy.Publisher(topic_name + type_name + "_trajectory_controller/command", JointTrajectory, queue_size=1)
    rospy.sleep(3.0)

    traj_record = np.load(os.path.join(os.path.dirname(__file__), "trajectory2.npy"), allow_pickle=True)[()]

    traj = JointTrajectory()

    traj.joint_names.append(joint_prefix + "_joint_1")
    traj.joint_names.append(joint_prefix + "_joint_2")
    traj.joint_names.append(joint_prefix + "_joint_3")
    traj.joint_names.append(joint_prefix + "_joint_4")
    traj.joint_names.append(joint_prefix + "_joint_5")
    traj.joint_names.append(joint_prefix + "_joint_6")
    traj.joint_names.append(joint_prefix + "_joint_7")

    traj_point_goal = JointTrajectoryPoint()
    traj_point_goal.positions = np.zeros(7)
    traj_point_goal.velocities = np.zeros(7)
    traj_point_goal.accelerations = np.zeros(7)
    traj.points.append(deepcopy(traj_point_goal))

    traj_point_goal.positions[:6] = traj_record['q'][0]
    print(traj_point_goal.positions)
    traj_point_goal.time_from_start = rospy.Time(2.0)
    traj.points.append(deepcopy(traj_point_goal))
    print(traj.points)
    traj.header.stamp = rospy.Time.now()
    #traj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
    cmdPub.publish(traj)

    print("move to base command sent")

    time.sleep(2.0)
    #assert False

    traj.points.clear()
    for i in range(traj_record['t'].shape[0]):
        traj_point_goal.positions[:6] = traj_record['q'][i]
        traj_point_goal.velocities[:6] = traj_record['dq'][i]
        traj_point_goal.accelerations[:6] = traj_record['ddq'][i]
        traj_point_goal.time_from_start = rospy.Duration(traj_record['t'][i])
        traj.points.append(deepcopy(traj_point_goal))

    for i in range(1, traj_record['t_r'].shape[0]):
        traj_point_goal.positions[:6] = traj_record['q_r'][i]
        traj_point_goal.velocities[:6] = traj_record['dq_r'][i]
        traj_point_goal.accelerations[:6] = traj_record['ddq_r'][i]
        traj_point_goal.time_from_start = rospy.Duration(traj_record['t_r'][i] + traj_record['t'][-1])
        traj.points.append(deepcopy(traj_point_goal))

    traj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
    cmdPub.publish(traj)

    print("hitting move command sent")