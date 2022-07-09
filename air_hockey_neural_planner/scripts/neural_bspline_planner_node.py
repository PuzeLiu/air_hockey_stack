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

SCRIPT_DIR = os.path.dirname(__file__)
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
PLANNING_MODULE_DIR = os.path.join(SCRIPT_DIR, "manifold_planning")
sys.path.append(PLANNING_MODULE_DIR)
from air_hockey_neural_planner.msg import PlannerRequest
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3
from iiwas_control.msg import BsplineTrajectoryMsg, BsplineSegmentMsg
from planner_request_utils import unpack_planner_request
from manifold_planning.utils.bspline import BSpline
from manifold_planning.utils.spo import StartPointOptimizer
from manifold_planning.utils.constants import UrdfModels, Limits, Base
from manifold_planning.utils.model import load_model_boundaries, load_model_hpo, model_inference


def print_(x, N=5):
    for i in range(N):
        print()
    print(x)
    for i in range(N):
        print()


class Trajectory:
    def __init__(self, q, dq, ddq, t):
        self.q = q
        self.dq = dq
        self.ddq = ddq
        self.t = t
        self.init_time = 0

    def append(self, q, dq, ddq, t):
        self.q = np.concatenate([self.q, q], axis=0)
        self.dq = np.concatenate([self.dq, dq], axis=0)
        self.ddq = np.concatenate([self.ddq, ddq], axis=0)
        self.t = np.concatenate([self.t, t + self.t[-1]], axis=0)

    def set_init_time(self, t):
        self.init_time = t

    def sample(self, t):
        t -= self.init_time
        idx = np.searchsorted(self.t, t, side="left")
        return self.q[idx], self.dq[idx], self.ddq[idx]


class NeuralPlannerNode:
    def __init__(self):
        rospy.init_node("neural_planner_node", anonymous=True)
        front_controller_type = rospy.get_param("~front_controllers", "bspline_adrc_joint_trajectory_controller")
        back_controller_type = rospy.get_param("~back_controllers", "bspline_adrc_joint_trajectory_controller")
        planner_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/planner_path"))
        ik_hitting_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/ik_hitting_path"))
        self.planning_request_subscriber = rospy.Subscriber("/neural_planner/plan_trajectory", PlannerRequest,
                                                            self.compute_trajectory)
        self.replanning_request_subscriber = rospy.Subscriber("/neural_planner/replan_trajectory", PlannerRequest,
                                                              self.replan_trajectory)
        self.iiwa_front_publisher = rospy.Publisher(f"/iiwa_front/{front_controller_type}/bspline",
                                                    BsplineTrajectoryMsg,
                                                    queue_size=5)
        self.cartesian_front_publisher = rospy.Publisher(f"/iiwa_front/cartesian_trajectory", MultiDOFJointTrajectory,
                                                         queue_size=5)
        self.urdf_path = os.path.join(PLANNING_MODULE_DIR, UrdfModels.striker)

        N = 15
        self.bsp = BSpline(N, num_T_pts=64)
        self.bspt = BSpline(20, num_T_pts=64)
        print("Bspline initialized")
        self.spo = StartPointOptimizer(self.urdf_path)
        self.planner_model = load_model_boundaries(planner_path, N, 3, 2, self.bsp, self.bspt)
        print("striker model loaded")
        self.ik_hitting_model = load_model_hpo(ik_hitting_path)
        print("ik hitting model loaded")
        self.pino_model = pino.buildModelFromUrdf(self.urdf_path)
        self.pino_data = self.pino_model.createData()
        self.joint_idx = self.pino_model.getFrameId("F_striker_tip")
        print("pino model loaded")
        rospy.sleep(2.)
        print("node loaded")
        self.actual_trajectory = None

    def replan_trajectory(self, msg):
        print("ROS TIME", rospy.Time.now().to_sec())
        t0 = perf_counter()
        if self.actual_trajectory is None:
            print("CANNOT REPLAN, THERE IS NO ACTUAL TRAJECTORY")
            return False
        if msg.tactic == 0:
            #plannig_time = 0.038
            plannig_time = 0.055
        elif msg.tactic == 1:
            plannig_time = 0.040
        communication_delays = 0.006
        traj_time = rospy.Time.now().to_sec() + plannig_time + communication_delays
        # traj_time = msg.header.stamp.to_sec() + plannig_time + communication_delays
        #print("REPLAN!!!")
        q_0, q_dot_0, q_ddot_0 = self.actual_trajectory.sample(traj_time)
        q_0 = np.concatenate([q_0, np.zeros(1)])
        q_dot_0 = np.concatenate([q_dot_0, np.zeros(1)])
        q_ddot_0 = np.concatenate([q_ddot_0, np.zeros(1)])
        #print(msg.q_0)
        #print(q_0)
        msg.q_0 = q_0
        msg.q_dot_0 = q_dot_0
        msg.q_ddot_0 = q_ddot_0
        t1 = perf_counter()
        print("REPLANNING TIME:", t1 - t0)
        self.compute_trajectory(msg)

    def compute_trajectory(self, msg):
        t0 = perf_counter()
        tactic, x_hit, y_hit, th_hit, q_0, q_dot_0, q_ddot_0, x_end, y_end = unpack_planner_request(msg)

        list_q = []
        list_t = []
        list_q_cps = []
        list_t_cps = []

        if tactic == 0:  # HIT
            q_d, q_dot_d, xyz = self.get_hitting_configuration(x_hit, y_hit, th_hit)
            d = np.concatenate([q_0, q_d, [x_hit, y_hit, th_hit], q_dot_0, q_ddot_0, q_dot_d], axis=-1)[np.newaxis]
            d = d.astype(np.float32)
            q, dq, ddq, t, q_cps, t_cps = model_inference(self.planner_model, d, self.bsp, self.bspt)
            self.actual_trajectory = Trajectory(q, dq, ddq, t)
            list_q.append(q)
            list_t.append(t)
            list_q_cps.append(q_cps)
            list_t_cps.append(t_cps)
            d_ret = np.concatenate([q_d, Base.configuration, [0.], Base.position, dq[-1], [0.], ddq[-1], [0.] * 8],
                                   axis=-1)[np.newaxis]
            d_ret = d_ret.astype(np.float32)
            qr, dqr, ddqr, tr, qr_cps, tr_cps = model_inference(self.planner_model, d_ret, self.bsp, self.bspt)
            self.actual_trajectory.append(qr, dqr, ddqr, tr)
            list_q.append(qr)
            list_t.append(tr)
            list_q_cps.append(qr_cps)
            list_t_cps.append(tr_cps)
        elif tactic == 1:  # MOVE
            p_d = np.array([x_end, y_end, Base.position[-1]])
            q_d = self.spo.solve(p_d)
            q_dot_d = np.zeros((7,))
            d = np.concatenate([q_0, q_d, p_d, q_dot_0, q_ddot_0, q_dot_d], axis=-1)[np.newaxis]
            q, dq, ddq, t, q_cps, t_cps = model_inference(self.planner_model, d, self.bsp, self.bspt)
            self.actual_trajectory = Trajectory(q, dq, ddq, t)
            list_q.append(q)
            list_t.append(t)
            list_q_cps.append(q_cps)
            list_t_cps.append(t_cps)

        tros = rospy.Time.now().to_sec()
        self.publish_joint_trajectory(list_q_cps, list_t_cps)
        t1 = perf_counter()
        print("PLANNING TIME: ", t1 - t0)
        print("ROS TIME", tros)
        self.publish_cartesian_trajectory(list_q, list_t)

    def get_hitting_configuration(self, xk, yk, thk):
        qk = self.ik_hitting_model(np.array([xk, yk, thk])[np.newaxis])
        q = np.concatenate([qk.numpy()[0], np.zeros(3)], axis=-1)
        pino.forwardKinematics(self.pino_model, self.pino_data, q)
        xyz_pino = copy(self.pino_data.oMi[-1].translation)
        #J = pino.computeJointJacobians(self.pino_model, self.pino_data, q)
        J = pino.computeFrameJacobian(self.pino_model, self.pino_data, q, self.joint_idx, pino.LOCAL_WORLD_ALIGNED)[:3, :6]
        pinvJ = np.linalg.pinv(J)
        #pinv63J = pinvJ[:6, :3]
        q_dot = (pinvJ @ np.array([np.cos(thk), np.sin(thk), 0])[:, np.newaxis])[:, 0]
        max_mul = np.max(np.abs(q_dot) / Limits.q_dot)
        qdotk = q_dot / max_mul
        return q[:7], qdotk, xyz_pino

    def publish_joint_trajectory(self, qs, ts):
        assert len(qs) == len(ts)

        # print("SIZE:", qs[0].shape)

        iiwa_front_bspline = BsplineTrajectoryMsg()
        for i in range(len(qs)):
            iiwa_front_bspline_hitting = BsplineSegmentMsg()
            iiwa_front_bspline_hitting.q_control_points = qs[i].flatten()
            iiwa_front_bspline_hitting.t_control_points = ts[i].flatten()
            iiwa_front_bspline.segments.append(iiwa_front_bspline_hitting)
        iiwa_front_bspline.header.stamp = rospy.Time.now()  # + rospy.Duration(0.01)
        self.actual_trajectory.set_init_time(iiwa_front_bspline.header.stamp.to_sec())
        self.iiwa_front_publisher.publish(iiwa_front_bspline)

    def publish_cartesian_trajectory(self, qs, ts):
        assert len(qs) == len(ts)
        xyz = []
        t = []
        t_offset = 0.
        for i in range(len(qs)):
            for j, qi in enumerate(np.pad(qs[i], [[0, 0], [0, 3]], mode='constant')):
                pino.forwardKinematics(self.pino_model, self.pino_data, qi)
                xyz_pino = copy(self.pino_data.oMi[-1].translation)
                xyz.append(xyz_pino)
                t.append(ts[i][j])
            t_offset += ts[i][-1]

        cart_traj = MultiDOFJointTrajectory()
        cart_traj.header.frame_id = 'F_link_0'
        cart_traj.header.stamp = rospy.Time.now()
        for i in range(len(xyz)):
            point = MultiDOFJointTrajectoryPoint()
            v3 = Vector3(*xyz[i])
            geometry = Transform(translation=v3)
            point.time_from_start = rospy.Duration(t[i])
            point.transforms.append(geometry)
            cart_traj.points.append(point)

        self.cartesian_front_publisher.publish(cart_traj)


if __name__ == '__main__':
    node = NeuralPlannerNode()
    rospy.spin()
