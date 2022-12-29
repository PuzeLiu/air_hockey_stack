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


SCRIPT_DIR = os.path.dirname(__file__)
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
PLANNING_MODULE_DIR = os.path.join(SCRIPT_DIR, "manifold_planning")
sys.path.append(PLANNING_MODULE_DIR)
from air_hockey_msgs.msg import PlannerRequest, PlannerStatus
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3
from iiwas_control.msg import BsplineTrajectoryMsg, BsplineSegmentMsg
from air_hockey_msgs.srv import GetHittingState
from planner_request_utils import unpack_planner_request
from manifold_planning.utils.bspline import BSpline
from manifold_planning.utils.spo import StartPointOptimizer
from manifold_planning.utils.constants import UrdfModels, Limits, Base
from manifold_planning.utils.model import load_model_hitting, model_inference
from manifold_planning.utils.feasibility import check_if_plan_valid
from manifold_planning.utils.hpo_interface import get_hitting_configuration_opt


def print_(x, N=5):
    for i in range(N):
        print()
    print(x)
    for i in range(N):
        print()


class Trajectory:
    def __init__(self, q, dq, ddq, t, q_cps, t_cps):
        self.q = q
        self.dq = dq
        self.ddq = ddq
        self.t = t
        self.init_time = 0
        self.update_samplers()
        self.q_list = [q]
        self.t_list = [t]
        self.q_cps_list = [q_cps]
        self.t_cps_list = [t_cps]

    def update_samplers(self):
        self.qs = interp1d(self.t, self.q, axis=0)
        self.dqs = interp1d(self.t, self.dq, axis=0)
        self.ddqs = interp1d(self.t, self.ddq, axis=0)

    def append(self, q, dq, ddq, t, q_cps, t_cps):
        self.q = np.concatenate([self.q, q], axis=0)
        self.dq = np.concatenate([self.dq, dq], axis=0)
        self.ddq = np.concatenate([self.ddq, ddq], axis=0)
        self.t = np.concatenate([self.t, t + self.t[-1]], axis=0)
        self.update_samplers()
        self.q_list.append(q)
        self.t_list.append(t)
        self.q_cps_list.append(q_cps)
        self.t_cps_list.append(t_cps)

    def set_init_time(self, t):
        self.init_time = t

    def get_hitting(self):
        idx = len(self.q_list[0]) - 1
        return self.t[idx], self.q[idx], self.dq[idx], self.ddq[idx]

    def sample(self, t):
        t -= self.init_time
        t = np.maximum(t, 0.)
        idx = np.searchsorted(self.t, t, side="left")
        print("TIME QUERY: ", t)
        print("TIME IDX: ", self.t[idx])
        print("Q IDX:", self.q[idx])
        print("Q INTERP:", self.qs(t))
        return self.qs(t), self.dqs(t), self.dqs(t)
        # return self.q[idx], self.dq[idx], self.ddq[idx]


class NeuralPlannerNode:
    def __init__(self):
        rospy.init_node("neural_planner_node", anonymous=True)
        front_controller_type = rospy.get_param("~front_controllers", "bspline_adrc_joint_trajectory_controller")
        back_controller_type = rospy.get_param("~back_controllers", "bspline_adrc_joint_trajectory_controller")
        print(front_controller_type)
        planner_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/planner_path"))
        self.planning_request_subscriber = rospy.Subscriber("/neural_planner/plan_trajectory", PlannerRequest,
                                                            self.compute_trajectory)
        self.replanning_request_subscriber = rospy.Subscriber("/neural_planner/replan_trajectory", PlannerRequest,
                                                              self.replan_trajectory)
        self.iiwa_front_publisher = rospy.Publisher(f"/iiwa_front/{front_controller_type}/bspline",
                                                    BsplineTrajectoryMsg,
                                                    queue_size=5)
        self.cartesian_front_publisher = rospy.Publisher(f"/iiwa_front/cartesian_trajectory", MultiDOFJointTrajectory,
                                                         queue_size=5)
        self.planner_status_publisher = rospy.Publisher(f"/neural_planner/status", PlannerStatus, queue_size=5)
        self.urdf_path = os.path.join(PLANNING_MODULE_DIR, UrdfModels.striker)
        rospy.wait_for_service('/iiwa_front/get_hitting_state')
        self.get_hitting_state = rospy.ServiceProxy('/iiwa_front/get_hitting_state', GetHittingState)

        self.dim_q_control_points = 6
        self.num_q_control_points = 15
        self.num_t_control_points = 20
        self.bsp = BSpline(self.num_q_control_points, num_T_pts=64)
        self.bspt = BSpline(self.num_t_control_points, num_T_pts=64)
        print("Bspline initialized")
        self.spo = StartPointOptimizer(self.urdf_path)
        self.planner_model = load_model_hitting(planner_path, self.num_q_control_points, self.bsp, self.bspt)
        print("striker model loaded")
        self.po = StartPointOptimizer(self.urdf_path)
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
            # plannig_time = 0.038
            # plannig_time = 0.032
            plannig_time = 0.06
        elif msg.tactic == 1:
            plannig_time = 0.040
        communication_delays = 0.006
        time_offset = plannig_time + communication_delays
        # traj_time = rospy.Time.now().to_sec() + time_offset
        traj_time = msg.header.stamp.to_sec() + time_offset
        # print("REPLAN!!!")
        q_0, q_dot_0, q_ddot_0 = self.actual_trajectory.sample(traj_time - 0.003)
        q_0 = np.concatenate([q_0, np.zeros(1)])
        q_dot_0 = np.concatenate([q_dot_0, np.zeros(1)])
        q_ddot_0 = np.concatenate([q_ddot_0, np.zeros(1)])
        # print(msg.q_0)
        print("REPLANNING Q0", q_0)
        msg.q_0 = q_0
        msg.q_dot_0 = q_dot_0
        msg.q_ddot_0 = q_ddot_0
        t1 = perf_counter()
        print("REPLANNING TIME:", t1 - t0)
        print("TIME OFFSET:", time_offset   )
        self.compute_trajectory(msg, traj_time, time_offset)

    def compute_trajectory(self, msg, traj_time=None, time_offset=0.):
        tactic, x_hit, y_hit, th_hit, q_0, q_dot_0, q_ddot_0, x_end, y_end, expected_time, expected_velocity = unpack_planner_request(msg)
        #print("TH HIT:", th_hit)

        v_mul = 1.0
        if tactic == 0:  # HIT
            #q_d, q_dot_d, xyz = self.get_hitting_configuration(x_hit, y_hit, th_hit)
            r = self.get_hitting_state(x_hit, y_hit, th_hit)
            q_d = np.array(r.q)
            q_dot_d = np.array(r.q_dot)
            magnitude = r.magnitude
            #print("EXPECTED VEL: ", expected_velocity)
            #print("MAGNITUDE: ", magnitude)
            if expected_velocity > 0. and expected_velocity < magnitude:
                q_dot_d = q_dot_d / magnitude * expected_velocity
            t0 = perf_counter()
            d = np.concatenate([q_0, q_d, [x_hit, y_hit, th_hit], q_dot_0, q_ddot_0, q_dot_d * v_mul], axis=-1)[np.newaxis]
            d = d.astype(np.float32)
            q, dq, ddq, t, q_cps, t_cps = model_inference(self.planner_model, d, self.bsp, self.bspt)
            #q, dq, ddq, t, q_cps, t_cps = model_inference(self.planner_model, d, self.bsp, self.bspt, expected_time=t[-1]*1.5)
            planning_time = 0.08
            traj_and_plan_time = t[-1] + planning_time
            #print("TRAJ TIME:", t[-1])
            #print("TRAJ AND PLAN TIME:", traj_and_plan_time)
            #print("EXPECTED_TIME:", expected_time)
            if expected_time > 0.:
                if traj_and_plan_time < expected_time:
                    if traj_time is None:
                        ttw = expected_time - traj_and_plan_time
                    else:
                        print("REPLANNING")
                        ttw = expected_time - t[-1] - time_offset
                        # robot is not moving
                    print("QDOT:", np.sum(np.abs(q_dot_0)))
                    if np.sum(np.abs(q_dot_0)) < 0.1:
                        print("TIME TO WAIT:", ttw)
                        traj_time = msg.header.stamp.to_sec() + np.maximum(ttw - 0.00, 0.)
                    else:
                        traj_and_offset_time = t[-1] + time_offset
                        print("TRAJ AND OFFSET TIME:", traj_and_offset_time)
                        if traj_and_offset_time < expected_time:
                            print("REPLANNING WHILE MOVING AND TOO MUCH TIME")
                            requested_time = expected_time - time_offset
                            print("REQ TIME:", requested_time)
                            ratio = requested_time / t[-1]
                            print("RATIO:", ratio)
                            #t_cps /= ratio
                            #t *= ratio
                elif traj_and_plan_time < 1.2*expected_time:
                    print("TRY TO ACT EVEN IT IS TOO LATE")
                    #traj_and_offset_time = t[-1] + time_offset
                    #print("TRAJ AND OFFSET TIME:", traj_and_offset_time)
                else:
                    return False
            t1 = perf_counter()
            self.actual_trajectory = Trajectory(q, dq, ddq, t, q_cps, t_cps)
            t2 = perf_counter()
            d_ret = np.concatenate([q_d, Base.configuration, [0.], Base.position, dq[-1], [0.], ddq[-1], [0.] * 8],
                                   axis=-1)[np.newaxis]
            d_ret = d_ret.astype(np.float32)
            qr, dqr, ddqr, tr, qr_cps, tr_cps = model_inference(self.planner_model, d_ret, self.bsp, self.bspt)
            t3 = perf_counter()
            self.actual_trajectory.append(qr, dqr, ddqr, tr, qr_cps, tr_cps)
            total_planning_time = t1 - t0# + t3 - t2
            print("TOTAL PLANNIVG TIME: ", total_planning_time)
            self.publish_planner_status(total_planning_time)
        elif tactic == 1:  # MOVE
            p_d = np.array([x_end, y_end, Base.position[-1]])
            q_d = self.spo.solve(p_d)
            q_dot_d = np.zeros((7,))
            d = np.concatenate([q_0, q_d, p_d, q_dot_0, q_ddot_0, q_dot_d], axis=-1)[np.newaxis]
            d = d.astype(np.float32)
            q, dq, ddq, t, q_cps, t_cps = model_inference(self.planner_model, d, self.bsp, self.bspt)
            print("TRAJ TIME:", t[-1])
            self.actual_trajectory = Trajectory(q, dq, ddq, t, q_cps, t_cps)
        elif tactic == 2:
            expected_time = 0.6
            x_1 = 0.75
            y_1 = -0.15
            th_1 = 0.5
            x_2 = 0.75
            y_2 = 0.15
            th_2 = -0.5
            p_d1 = np.array([x_1, y_1, Base.position[-1]])
            p_d2 = np.array([x_2, y_2, Base.position[-1]])
            mul = 0.5
            v1 = mul * np.array([np.cos(th_1), np.sin(th_1), 0.])
            v2 = mul * np.array([np.cos(th_2), np.sin(th_2), 0.])

            q_d1 = self.po.solve(p_d1)
            q_d2 = self.po.solve(p_d2)

            def get_velocity(q, v_xyz):
                q = np.pad(q, (0, 9 - q.shape[0]), mode='constant')
                idx_ = self.pino_model.getFrameId("F_striker_tip")
                J = pino.computeFrameJacobian(self.pino_model, self.pino_data, q, idx_, pino.LOCAL_WORLD_ALIGNED)[:3, :6]
                pinvJ = np.linalg.pinv(J)
                q_dot = pinvJ @ v_xyz
                return q_dot

            q_dot_d1 = get_velocity(q_d1, v1)
            q_dot_d2 = get_velocity(q_d2, v2)
            q_dot_d1 = np.pad(q_dot_d1, [[0, 1]], mode='constant')
            q_dot_d2 = np.pad(q_dot_d2, [[0, 1]], mode='constant')
            q_ddot_0 = np.zeros((7,))
            d = np.concatenate([q_0, q_d1, p_d1, q_dot_0, q_ddot_0, q_dot_d1], axis=-1)[np.newaxis]
            d = d.astype(np.float32)
            q, dq, ddq, t, q_cps, t_cps = model_inference(self.planner_model, d, self.bsp, self.bspt,
                                                          expected_time=expected_time)
            self.actual_trajectory = Trajectory(q, dq, ddq, t, q_cps, t_cps)
            d = np.concatenate([q_d1, q_d2, p_d2, q_dot_d1, ddq[-1], [0.], q_dot_d2], axis=-1)[np.newaxis]
            d = d.astype(np.float32)
            q, dq, ddq, t, q_cps, t_cps = model_inference(self.planner_model, d, self.bsp, self.bspt,
                                                          expected_time=expected_time)
            t1 = (q, dq, ddq, t, q_cps, t_cps)
            d = np.concatenate([q_d2, q_d1, p_d1, q_dot_d2, ddq[-1], [0.], q_dot_d1], axis=-1)[np.newaxis]
            d = d.astype(np.float32)
            q, dq, ddq, t, q_cps, t_cps = model_inference(self.planner_model, d, self.bsp, self.bspt,
                                                          expected_time=expected_time)
            t2 = (q, dq, ddq, t, q_cps, t_cps)
            for i in range(5):
                self.actual_trajectory.append(*t1)
                self.actual_trajectory.append(*t2)

        tros = rospy.Time.now().to_sec()
        self.publish_joint_trajectory(self.actual_trajectory.q_cps_list, self.actual_trajectory.t_cps_list, traj_time)
        print("PLANNING TIME: ", t1 - t0)
        print("ROS TIME", tros)
        self.publish_cartesian_trajectory(self.actual_trajectory.q_list, self.actual_trajectory.t_list)

    def get_hitting_configuration(self, xk, yk, thk):
        qk, q_dot_k = get_hitting_configuration_opt(xk, yk, Base.position[-1], thk)
        if qk is None or q_dot_k is None:
            return None, None
        q = np.concatenate([qk, np.zeros(2)], axis=-1)
        pino.forwardKinematics(self.pino_model, self.pino_data, q)
        xyz_pino = self.pino_data.oMi[-1].translation
        return q[:7], np.array(q_dot_k[:7]), xyz_pino
        #qk = self.ik_hitting_model(np.array([xk, yk, thk])[np.newaxis]).numpy()[0]
        #q = np.concatenate([qk, np.zeros(3)], axis=-1)
        #pino.forwardKinematics(self.pino_model, self.pino_data, q)
        #xyz_pino = copy(self.pino_data.oMi[-1].translation)
        ## J = pino.computeJointJacobians(self.pino_model, self.pino_data, q)
        #J = pino.computeFrameJacobian(self.pino_model, self.pino_data, q, self.joint_idx, pino.LOCAL_WORLD_ALIGNED)[:3,
        #    :6]
        #pinvJ = np.linalg.pinv(J)
        ## pinv63J = pinvJ[:6, :3]
        #q_dot = (pinvJ @ np.array([np.cos(thk), np.sin(thk), 0])[:, np.newaxis])[:, 0]
        #max_mul = np.max(np.abs(q_dot) / Limits.q_dot)
        #qdotk = q_dot / max_mul
        #return q[:7], qdotk, xyz_pino

    def publish_joint_trajectory(self, qs, ts, traj_time=None):
        assert len(qs) == len(ts)

        # print("SIZE:", qs[0].shape)

        iiwa_front_bspline = BsplineTrajectoryMsg()
        for i in range(len(qs)):
            iiwa_front_bspline_hitting = BsplineSegmentMsg()
            iiwa_front_bspline_hitting.q_control_points = qs[i].flatten()
            iiwa_front_bspline_hitting.t_control_points = ts[i].flatten()
            iiwa_front_bspline_hitting.dim_q_control_points = self.dim_q_control_points
            iiwa_front_bspline_hitting.num_q_control_points = self.num_q_control_points
            iiwa_front_bspline_hitting.num_t_control_points = self.num_t_control_points
            iiwa_front_bspline.segments.append(iiwa_front_bspline_hitting)
        if traj_time is None:
            iiwa_front_bspline.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        else:
            iiwa_front_bspline.header.stamp = rospy.Time(traj_time)
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

    def publish_planner_status(self, planning_time):
        q = self.actual_trajectory.q
        dq = self.actual_trajectory.dq
        ddq = self.actual_trajectory.ddq
        q_ = np.pad(q, [[0, 0], [0, 3]])
        dq_ = np.pad(dq, [[0, 0], [0, 3]])
        ddq_ = np.pad(ddq, [[0, 0], [0, 3]])
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
        valid = check_if_plan_valid(xyz, q, dq, ddq, torque)
        planner_status = PlannerStatus()
        planner_status.success = valid
        planner_status.planning_time = planning_time * 1000.
        t_hit, q_hit, dq_hit, _ = self.actual_trajectory.get_hitting()
        planner_status.planned_motion_time = self.actual_trajectory.t[-1]
        planner_status.planned_hitting_time = t_hit
        planner_status.planned_hit_joint_velocity = dq_hit
        q_hit = np.concatenate([q_hit, np.zeros(3)], axis=-1)
        pino.forwardKinematics(self.pino_model, self.pino_data, q_hit)
        J = pino.computeFrameJacobian(self.pino_model, self.pino_data, q_hit, self.joint_idx,
                                      pino.LOCAL_WORLD_ALIGNED)[:3, :6]
        planner_status.planned_hit_cartesian_velocity = J @ dq_hit
        self.planner_status_publisher.publish(planner_status)


if __name__ == '__main__':
    node = NeuralPlannerNode()
    rospy.spin()
