#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tf
import numpy as np

import torch
from air_hockey.robots.iiwa.kinematics_torch import KinematicsTorch
from hard_coded_policy.traj_generator import Poly3
from TrajOpt.cart_null_space.bezier_interpolation_nullspace import NullSpaceTrajectory
import matplotlib.pyplot as plt


def goto(goal, t_f=3.0):
    q_0 = q_cur
    traj = Poly3(q_0, goal, 0, 0, t_f)
    msg = Float64MultiArray()
    for t in np.arange(0, t_f, rate.sleep_dur.to_sec()):
        msg.data = traj(t)[0]
        cmd_pubisher.publish(msg)
        rate.sleep()

def joint_state_cb(data):
    if data.name[0] == "F_joint_1":
        global q_cur
        q_cur = np.array(data.position)

if __name__ == '__main__':
    rospy.init_node("hit_static", anonymous=True)
    rate = rospy.Rate(100)

    tf_listener = tf.TransformListener()

    cmd_pubisher = rospy.Publisher("iiwa_front/joint_position_controller/command", Float64MultiArray, queue_size=1)
    rospy.Subscriber("iiwa_front/joint_states", JointState, joint_state_cb)

    table_height = 0.1915

    # Get initial position
    while True:
        msg = rospy.wait_for_message("iiwa_front/joint_states", JointState)
        if msg.name[0] == "F_joint_1":
            q_cur = np.array(msg.position)
            break

    tcp_pos = torch.tensor([0., 0., 0.59])
    tcp_quat = torch.tensor([1., 0., 0., 0.])
    kinematics = KinematicsTorch(tcp_pos=tcp_pos,
                                 tcp_quat=tcp_quat)

    # Goto initial position
    pose_init = torch.tensor([0.45, 0., table_height, 0., 0., 1., 0.])
    gc = torch.tensor([1., -1., 1.])
    res, q_init = kinematics.inverse_kinematics(pose_init, torch.tensor([0.]), gc)
    goto(q_init.detach().numpy(), 3.0)

    while not rospy.is_shutdown():
        charac = input("press Key to continue (r)eset/ (g)oto:\n")
        if charac =="g":
            q_0 = torch.tensor(q_cur)
            x_f = torch.tensor([0.7, 0.2, table_height])
            v_f = torch.tensor([1.6, 0.0, 0])
            # Initialize Trajectory planner
            null_traj = NullSpaceTrajectory(q_0=q_0, x_f=x_f, v_f=v_f, table_height=table_height, kinematics=kinematics)
            traj_q, traj_x, t = null_traj.generate_trajectory(rate.sleep_dur.to_sec())
            msg_pub = Float64MultiArray()
            # null_traj.plot()
            for traj_i in traj_q:
                msg_pub.data = traj_i.detach().numpy()[0]
                cmd_pubisher.publish(msg_pub)
                rate.sleep()
        elif charac == "r":
            goto(q_init.detach().numpy(), 5.0)
        else:
            continue