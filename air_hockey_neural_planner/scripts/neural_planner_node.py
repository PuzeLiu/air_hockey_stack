#!/usr/bin/env python
import rospy
from scipy.interpolate import interp1d
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import os
import sys
import inspect
import numpy as np
from copy import copy
import pinocchio as pino
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(__file__)
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
PLANNING_MODULE_DIR = os.path.join(SCRIPT_DIR, "manifold_planning")
sys.path.append(PLANNING_MODULE_DIR)

from air_hockey_neural_planner.msg import PlannerRequest
from planner_request_utils import unpack_planner_request
from manifold_planning.utils.bspline import BSpline
from manifold_planning.utils.constants import UrdfModels, Limits, Base
from manifold_planning.utils.model import load_model_boundaries, load_model_hpo, model_inference


def print_(x, N=5):
    for i in range(N):
        print()
    print(x)
    for i in range(N):
        print()


class NeuralPlannerNode:
    def __init__(self):
        rospy.init_node("neural_planner_node", anonymous=True)
        front_controller_type = rospy.get_param("~front_controllers", "adrc_trajectory_controller")
        back_controller_type = rospy.get_param("~back_controllers", "adrc_trajectory_controller")
        planner_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/planner_path"))
        ik_hitting_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/ik_hitting_path"))
        self.planning_request_subscriber = rospy.Subscriber("/neural_planner/plan_trajectory", PlannerRequest,
                                                            self.compute_trajectory)
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

        N = 15
        self.bsp = BSpline(N)
        self.bspt = BSpline(20)
        print("Bspline initialized")
        self.planner_model = load_model_boundaries(planner_path, N, 3, 2, self.bsp, self.bspt)
        print("striker model loaded")
        self.ik_hitting_model = load_model_hpo(ik_hitting_path)
        print("ik hitting model loaded")
        self.urdf_path = os.path.join(PLANNING_MODULE_DIR, UrdfModels.striker)
        self.pino_model = pino.buildModelFromUrdf(self.urdf_path)
        self.pino_data = self.pino_model.createData()
        print("pino model loaded")

    def compute_trajectory(self, msg):
        x_d, y_d, th_d, q_0, q_dot_0, q_ddot_0 = unpack_planner_request(msg)
        q_d, q_dot_d, xyz = self.get_hitting_configuration(x_d, y_d, th_d)

        d = q_0.tolist() + q_d.tolist() + [x_d, y_d, th_d] + q_dot_0.tolist() + q_ddot_0.tolist() + q_dot_d.tolist()
        d = np.array(d)[np.newaxis]
        q, dq, ddq, t = model_inference(self.planner_model, d, self.bsp, self.bspt, uniform=True, freq=100)
        print_(q.shape)

        d_ret = q_d.tolist() + Base.configuration + [0.] + [0.65, 0.0, 0.0] + dq[-1].tolist() + [0.] + ddq[-1].tolist() + [
            0.] * 8
        d_ret = np.array(d_ret)[np.newaxis]
        q_r, dq_r, ddq_r, t_r = model_inference(self.planner_model, d_ret, self.bsp, self.bspt, uniform=True, freq=100)
        print_(q_r.shape)

        z = np.zeros_like(q)[:, :1]
        q = np.concatenate([q, z], axis=-1)
        dq = np.concatenate([dq, z], axis=-1)
        ddq = np.concatenate([ddq, z], axis=-1)
        z = np.zeros_like(q_r)[:, :1]
        q_r = np.concatenate([q_r, z], axis=-1)
        dq_r = np.concatenate([dq_r, z], axis=-1)
        ddq_r = np.concatenate([ddq_r, z], axis=-1)


        #plt.subplot(121)
        #for i in range(6):
        #    plt.plot(t, q[:, i], label=f"q_{i}")
        #plt.legend()
        #plt.subplot(122)
        #for i in range(6):
        #    plt.plot(t, dq[:, i], label=f"dq_{i}")
        #plt.legend()
        #plt.savefig("/home/piotr/Desktop/save.png")

        # t = np.concatenate([[0.], t[1:]], axis=0)
        #qi = interp1d(t, q, axis=0)
        #dqi = interp1d(t, dq, axis=0)
        #ddqi = interp1d(t, ddq, axis=0)

        #freq = 100
        #t_uniform = np.linspace(t[0], t[-1], int(freq * t[-1]))

        #qi_r = interp1d(t_r, q_r, axis=0)
        #dqi_r = interp1d(t_r, dq_r, axis=0)
        #ddqi_r = interp1d(t_r, ddq_r, axis=0)
        #t_r_uniform = np.linspace(t_r[1], t_r[-1], int(freq * t_r[-1]))

        #print(t_uniform.shape)
        #print(t_r_uniform.shape)

        t = t + 0.01

        self.iiwa_front_trajectory.points = []
        for i in range(0, q.shape[0]):
        #for i in t_uniform:
            traj_point_goal = JointTrajectoryPoint()
            #traj_point_goal.positions = qi(i)
            #traj_point_goal.velocities = dqi(i)
            #traj_point_goal.accelerations = ddqi(i)
            #traj_point_goal.time_from_start = rospy.Time(i)
            traj_point_goal.positions = q[i]
            traj_point_goal.velocities = dq[i]
            #traj_point_goal.accelerations = ddq[i]
            traj_point_goal.time_from_start = rospy.Time(t[i])
            self.iiwa_front_trajectory.points.append(traj_point_goal)
        for i in range(1, q_r.shape[0]):
        #for i in t_r_uniform:
            traj_point_goal = JointTrajectoryPoint()
            traj_point_goal.positions = q_r[i]
            traj_point_goal.velocities = dq_r[i]
            #traj_point_goal.accelerations = ddq_r[i]
            traj_point_goal.time_from_start = rospy.Time(t[-1] + t_r[i])
            #traj_point_goal.positions = qi_r(i)
            #traj_point_goal.velocities = dqi_r(i)
            #traj_point_goal.accelerations = ddqi_r(i)
            #traj_point_goal.time_from_start = rospy.Time(i + t[-1])
            self.iiwa_front_trajectory.points.append(traj_point_goal)
        print(len(self.iiwa_front_trajectory.points))
        self.iiwa_front_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        self.iiwa_front_publisher.publish(self.iiwa_front_trajectory)

        #self.iiwa_back_trajectory.points = [traj_point_goal]
        #self.iiwa_back_trajectory.header.stamp = rospy.Time.now()

        #self.iiwa_back_publisher.publish(self.iiwa_back_trajectory)

    def get_hitting_configuration(self, xk, yk, thk):
        qk = self.ik_hitting_model(np.array([xk, yk, thk])[np.newaxis])
        q = np.concatenate([qk.numpy()[0], np.zeros(3)], axis=-1)
        pino.forwardKinematics(self.pino_model, self.pino_data, q)
        xyz_pino = copy(self.pino_data.oMi[-1].translation)
        J = pino.computeJointJacobians(self.pino_model, self.pino_data, q)
        pinvJ = np.linalg.pinv(J)
        pinv63J = pinvJ[:6, :3]
        q_dot = (pinv63J @ np.array([np.cos(thk), np.sin(thk), 0])[:, np.newaxis])[:, 0]
        max_mul = np.max(np.abs(q_dot) / Limits.q_dot)
        qdotk = q_dot / max_mul
        return q[:7], qdotk, xyz_pino


if __name__ == '__main__':
    node = NeuralPlannerNode()
    rospy.spin()
