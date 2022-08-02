#!/usr/bin/env python
import os
import sys
from copy import copy
import numpy as np
import pinocchio as pino
import rospy

SCRIPT_DIR = os.path.dirname(__file__)
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
PLANNING_MODULE_DIR = os.path.join(SCRIPT_DIR, "manifold_planning")
sys.path.append(PLANNING_MODULE_DIR)

from air_hockey_msgs.srv import GetHittingState
from manifold_planning.utils.model import load_model_hpo
from manifold_planning.utils.constants import UrdfModels, Limits

class GHS():
    def __init__(self):
        rospy.init_node('get_hitting_state_node')
        s = rospy.Service('get_hitting_state', GetHittingState, self.response)
        ik_hitting_path = os.path.join(PACKAGE_DIR, rospy.get_param("/neural_planner/ik_hitting_path"))
        self.ik_hitting_model = load_model_hpo(ik_hitting_path)
        self.urdf_path = os.path.join(PLANNING_MODULE_DIR, UrdfModels.striker)
        self.pino_model = pino.buildModelFromUrdf(self.urdf_path)
        self.pino_data = self.pino_model.createData()
        self.joint_idx = self.pino_model.getFrameId("F_striker_tip")

    def response(self, req):
        print(req)
        xk = req.x
        yk = req.y
        thk = req.th
        qk = self.ik_hitting_model(np.array([xk, yk, thk])[np.newaxis]).numpy()[0]
        q = np.concatenate([qk, np.zeros(3)], axis=-1)
        pino.forwardKinematics(self.pino_model, self.pino_data, q)
        xyz_pino = copy(self.pino_data.oMi[-1].translation)
        # J = pino.computeJointJacobians(self.pino_model, self.pino_data, q)
        J = pino.computeFrameJacobian(self.pino_model, self.pino_data, q, self.joint_idx, pino.LOCAL_WORLD_ALIGNED)[:3,
            :6]
        pinvJ = np.linalg.pinv(J)
        # pinv63J = pinvJ[:6, :3]
        q_dot = (pinvJ @ np.array([np.cos(thk), np.sin(thk), 0])[:, np.newaxis])[:, 0]
        max_mul = np.max(np.abs(q_dot) / Limits.q_dot)
        qdotk = q_dot / max_mul
        return q[:7], qdotk, 1. / max_mul


if __name__ == "__main__":
    ghs = GHS()
    rospy.spin()
