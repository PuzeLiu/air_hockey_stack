#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
import numpy as np

import torch
from iiwas_kinematics.kinematics_torch import KinematicsTorch
from hard_coded_policy.traj_generator import Poly3
from TrajOpt.cart_null_space.bezier_interpolation_nullspace import NullSpaceTrajectory
import matplotlib.pyplot as plt
import time

class CmdPub:
    def __init__(self):
        self.cmd_publisher = rospy.Publisher("/iiwa_front/joint_trajectory_controller/command",
                                             JointTrajectory, queue_size=1)

    def goto(self, goal, t_f=3.0):
        # msg = Float64MultiArray()
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = ['F_joint_1', 'F_joint_2', 'F_joint_3', 'F_joint_4', 'F_joint_5', 'F_joint_6', 'F_joint_7']
        point = JointTrajectoryPoint()
        point.positions = goal
        point.velocities = np.zeros_like(goal)
        point.time_from_start = rospy.Duration.from_sec(t_f)
        msg.points.append(point)
        self.cmd_publisher.publish(msg)

def joint_state_cb(data):
    if data.name[0] == "F_joint_1":
        global q_cur
        q_cur = np.array(data.position)

if __name__ == '__main__':
    rospy.init_node("hit_static", anonymous=True)
    rate = rospy.Rate(100)

    tf_listener = tf.TransformListener()

    cmd_publisher = CmdPub()
    rospy.Subscriber("iiwa_front/joint_states", JointState, joint_state_cb)

    rospy.timer.sleep(1)

    table_height = 0.1915

    # Get initial position
    while True:
        msg = rospy.wait_for_message("iiwa_front/joint_states", JointState)
        if msg.name[0] == "F_joint_1":
            q_cur = np.array(msg.position)
            break

    tcp_pos = torch.tensor([0., 0., 0.5])
    tcp_quat = torch.tensor([1., 0., 0., 0.])
    kinematics = KinematicsTorch(tcp_pos=tcp_pos,
                                 tcp_quat=tcp_quat)

    # Goto initial position
    pose_init = torch.tensor([0.45, 0., table_height, 0., 0., 1., 0.])
    gc = torch.tensor([1., -1., 1.])
    res, q_init = kinematics.inverse_kinematics(pose_init, torch.tensor([-np.pi/3]), gc)
    cmd_publisher.goto(q_init.detach().numpy(), 3.0)
    rospy.sleep(3.0)

    while not rospy.is_shutdown():
        charac = input("press Key to continue (r)eset/ (g)oto:\n")
        if charac =="g":
            q_0 = torch.tensor(q_cur)
            x_f = torch.tensor([0.7, 0.2, table_height])
            v_f = torch.tensor([1.5, 0.0, 0])
            # Initialize Trajectory planner
            null_traj = NullSpaceTrajectory(q_0=q_0, x_f=x_f, v_f=v_f, table_height=table_height, kinematics=kinematics)
            traj_q, traj_x, t = null_traj.generate_trajectory(rate.sleep_dur.to_sec())
            null_traj.plot()
            traj_q = traj_q.detach().numpy()
            t = t.detach().numpy()

            for traj_i, t_i in zip(traj_q, t):
                cmd_publisher.goto(traj_i[0], rate.sleep_dur.to_sec())
                rate.sleep()
        elif charac == "r":
            cmd_publisher.goto(q_init.detach().numpy(), 3.0)
            rospy.sleep(3.0)
        else:
            continue