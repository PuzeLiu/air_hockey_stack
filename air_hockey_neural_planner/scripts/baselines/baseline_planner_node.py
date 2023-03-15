#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import os
import sys
import inspect
import numpy as np
from copy import copy
import pinocchio as pino
from time import perf_counter
from scipy.interpolate import interp1d

import matplotlib.pyplot as plt


BASELINES_DIR = os.path.dirname(__file__)
SCRIPTS_DIR = os.path.dirname(BASELINES_DIR)
PACKAGE_DIR = os.path.dirname(SCRIPTS_DIR)
PLANNING_MODULE_DIR = os.path.join(SCRIPTS_DIR, "manifold_planning")
sys.path.append(PLANNING_MODULE_DIR)
sys.path.append(SCRIPTS_DIR)

from air_hockey_msgs.msg import PlannerRequest, PlannerStatus
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import Transform, Vector3
from air_hockey_msgs.srv import GetHittingState
from planner_request_utils import unpack_planner_request
from manifold_planning.utils.constants import UrdfModels, Limits, Base
from manifold_planning.utils.feasibility import check_if_plan_valid
from mpcmpnet_planner import MPCMPNetPlanner
from sst_planner import SSTPlanner
from cbirrt_planner import CBiRRTPlanner
from nlopt_planner import NloptPlanner
from cbirrt_planner_vel import CBiRRTVelPlanner


def print_(x, N=5):
    for i in range(N):
        print()
    print(x)
    for i in range(N):
        print()


class NloptPlannerNode:
    def __init__(self):
        rospy.init_node("baselinet_planner_node", anonymous=True)
        #front_controller_type = rospy.get_param("~front_controllers", "bspline_adrc_joint_trajectory_controller")
        #back_controller_type = rospy.get_param("~back_controllers", "bspline_adrc_joint_trajectory_controller")
        front_controller_type = rospy.get_param("~front_controllers", "bspline_ff_joint_trajectory_controller")
        back_controller_type = rospy.get_param("~back_controllers", "bspline_ff_joint_trajectory_controller")
        planner_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/planner_path"))
        ik_hitting_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/ik_hitting_path"))
        self.planning_request_subscriber = rospy.Subscriber("/neural_planner/plan_trajectory", PlannerRequest,
                                                            self.compute_trajectory)
        self.iiwa_front_publisher = rospy.Publisher(f"/iiwa_front/{front_controller_type}/command",
                                                    JointTrajectory,
                                                    queue_size=5)
        self.cartesian_front_publisher = rospy.Publisher(f"/iiwa_front/cartesian_trajectory", MultiDOFJointTrajectory,
                                                         queue_size=5)
        self.planner_status_publisher = rospy.Publisher(f"/neural_planner/status", PlannerStatus, queue_size=5)
        self.urdf_path = os.path.join(PLANNING_MODULE_DIR, UrdfModels.striker)
        rospy.wait_for_service('/iiwa_front/get_hitting_state')
        self.get_hitting_state = rospy.ServiceProxy('/iiwa_front/get_hitting_state', GetHittingState)

        N = 15

        self.pino_model = pino.buildModelFromUrdf(self.urdf_path)
        self.pino_data = self.pino_model.createData()
        self.joint_idx = self.pino_model.getFrameId("F_striker_tip")
        print("pino model loaded")
        #self.planner = SSTPlanner()
        self.planner = MPCMPNetPlanner()
        #self.planner = NloptPlanner(N, self.pino_model, self.joint_idx)
        #self.planner = CBiRRTPlanner(N, self.pino_model, self.joint_idx)
        #self.planner = CBiRRTVelPlanner(N, self.pino_model, self.joint_idx)
        rospy.sleep(2.)
        print("node loaded")
        self.actual_trajectory = None

    def compute_trajectory(self, msg):
        tactic, x_hit, y_hit, th_hit, q_0, q_dot_0, q_ddot_0, x_end, y_end, expected_time, expected_velocity,\
        table_height = unpack_planner_request(msg)
        print("TH HIT:", th_hit)

        r = self.get_hitting_state(x_hit, y_hit, table_height, th_hit)
        q_d = np.array(r.q)
        q_dot_d = np.array(r.q_dot)
        q, dq, ddq, t, planning_time = self.planner.solve(q_0, q_dot_0, q_d, q_dot_d)

        self.publish_joint_trajectory(q, dq, ddq, t)
        self.publish_cartesian_trajectory(q, t)
        self.publish_planner_status(q, dq, ddq, t, planning_time)


    def publish_joint_trajectory(self, q, dq, ddq, t):
        iiwa_front_msg = JointTrajectory()
        print(q)
        print(t)
        for i in range(6):
            plt.subplot(321+i)
            plt.plot(q[:, i])
        plt.savefig("XD.png")
        for i in range(len(q)):
            pt = JointTrajectoryPoint()
            pt.positions = q[i].tolist() + [0.]
            if len(dq):
                pt.velocities = dq[i].tolist() + [0.]
                if len(ddq):
                    pt.accelerations = ddq[i].tolist() + [0.]
            pt.time_from_start = rospy.Duration(t[i])
            iiwa_front_msg.points.append(pt)
        iiwa_front_msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        iiwa_front_msg.joint_names = [f"F_joint_{i}" for i in range(1, 8)]
        self.iiwa_front_publisher.publish(iiwa_front_msg)

    def publish_cartesian_trajectory(self, qs, ts):
        assert len(qs) == len(ts)
        qs = np.pad(qs, [[0, 0], [0, 3]], mode='constant')
        cart_traj = MultiDOFJointTrajectory()
        cart_traj.header.frame_id = 'F_link_0'
        z = []
        for i in range(len(qs)):
            pino.forwardKinematics(self.pino_model, self.pino_data, qs[i])
            xyz_pino = copy(self.pino_data.oMi[-1].translation)
            z.append(xyz_pino[-1])
            point = MultiDOFJointTrajectoryPoint()
            v3 = Vector3(*xyz_pino)
            geometry = Transform(translation=v3)
            point.time_from_start = rospy.Duration(ts[i])
            point.transforms.append(geometry)
            cart_traj.points.append(point)
        plt.clf()
        plt.plot(z)
        plt.savefig("z.png")
        cart_traj.header.stamp = rospy.Time.now()
        self.cartesian_front_publisher.publish(cart_traj)

    def publish_planner_status(self, q, dq, ddq, t, planning_time):
        q_ = np.pad(q, [[0, 0], [0, 3]])
        dq_ = np.pad(dq, [[0, 0], [0, 3]]) if len(dq) else np.zeros_like(q_)
        ddq_ = np.pad(ddq, [[0, 0], [0, 3]]) if len(ddq) else np.zeros_like(q_)
        torque = []
        xyz = []
        for i in range(q.shape[0]):
            tau = pino.rnea(self.pino_model, self.pino_data, q_[i], dq_[i], ddq_[i])
            torque.append(tau)
            pino.forwardKinematics(self.pino_model, self.pino_data, q_[i])
            xyz_pino = copy(self.pino_data.oMi[-1].translation)
            xyz.append(xyz_pino)
        torque = np.array(torque)[:, :6]
        xyz = np.array(xyz)
        valid = check_if_plan_valid(xyz, q_, dq_, ddq_, torque)
        planner_status = PlannerStatus()
        planner_status.success = valid
        planner_status.planning_time = planning_time * 1000.
        t_hit, q_hit = t[-1], q[-1]
        dq_hit = dq[-1] if len(dq) else np.zeros_like(q_hit)
        planner_status.planned_motion_time = t_hit
        planner_status.planned_hitting_time = t_hit
        planner_status.planned_hit_joint_velocity = dq_hit
        q_hit = np.concatenate([q_hit, np.zeros(3)], axis=-1)
        pino.forwardKinematics(self.pino_model, self.pino_data, q_hit)
        J = pino.computeFrameJacobian(self.pino_model, self.pino_data, q_hit, self.joint_idx,
                                      pino.LOCAL_WORLD_ALIGNED)[:3, :6]
        planner_status.planned_hit_cartesian_velocity = J @ dq_hit
        self.planner_status_publisher.publish(planner_status)


if __name__ == '__main__':
    node = NloptPlannerNode()
    rospy.spin()
