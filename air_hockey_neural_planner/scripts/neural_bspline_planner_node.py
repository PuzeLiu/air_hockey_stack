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
from scipy.interpolate import interp1d

SCRIPT_DIR = os.path.dirname(__file__)
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
PLANNING_MODULE_DIR = os.path.join(SCRIPT_DIR, "manifold_planning")
sys.path.append(PLANNING_MODULE_DIR)

from air_hockey_neural_planner.msg import PlannerRequest
from iiwas_control.msg import BsplineTrajectoryMsg, BsplineSegmentMsg
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
        front_controller_type = rospy.get_param("~front_controllers", "bspline_adrc_joint_trajectory_controller")
        back_controller_type = rospy.get_param("~back_controllers", "bspline_adrc_joint_trajectory_controller")
        planner_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/planner_path"))
        ik_hitting_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/ik_hitting_path"))
        self.planning_request_subscriber = rospy.Subscriber("/neural_planner/plan_trajectory", PlannerRequest,
                                                            self.compute_trajectory)
        self.iiwa_front_publisher = rospy.Publisher(f"/iiwa_front/{front_controller_type}/bspline", BsplineTrajectoryMsg,
                                                    queue_size=5)
        #self.iiwa_back_publisher = rospy.Publisher(f"/iiwa_back/{back_controller_type}/command", JointTrajectory,
                                                   #queue_size=5)

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
        rospy.sleep(2.)
        print("node loaded")

    def compute_trajectory(self, msg):
        x_d, y_d, th_d, q_0, q_dot_0, q_ddot_0 = unpack_planner_request(msg)
        q_d, q_dot_d, xyz = self.get_hitting_configuration(x_d, y_d, th_d)

        d = q_0.tolist() + q_d.tolist() + [x_d, y_d, th_d] + q_dot_0.tolist() + q_ddot_0.tolist() + q_dot_d.tolist()
        d = np.array(d)[np.newaxis]
        q, dq, ddq, t = model_inference(self.planner_model, d, self.bsp, self.bspt, uniform=True, freq=2000)
        q_cps, t_cps = self.planner_model(d)

        d_ret = q_d.tolist() + Base.configuration + [0.] + [0.65, 0.0, 0.0] + dq[-1].tolist() + [0.] + ddq[-1].tolist() + [
            0.] * 8
        d_ret = np.array(d_ret)[np.newaxis]
        qr_cps, tr_cps = self.planner_model(d_ret)

        q_cps = q_cps.numpy()[0].flatten()
        t_cps = t_cps.numpy()[0].flatten()
        qr_cps = qr_cps.numpy()[0].flatten()
        tr_cps = tr_cps.numpy()[0].flatten()

        print("SIZE:", q_cps.shape)

        iiwa_front_bspline = BsplineTrajectoryMsg()
        iiwa_front_bspline_hitting = BsplineSegmentMsg()
        iiwa_front_bspline_hitting.q_control_points = q_cps
        iiwa_front_bspline_hitting.t_control_points = t_cps
        iiwa_front_bspline_returning = BsplineSegmentMsg()
        iiwa_front_bspline_returning.q_control_points = qr_cps
        iiwa_front_bspline_returning.t_control_points = tr_cps
        iiwa_front_bspline.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        iiwa_front_bspline.segments = [iiwa_front_bspline_hitting, iiwa_front_bspline_returning]
        self.iiwa_front_publisher.publish(iiwa_front_bspline)


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
