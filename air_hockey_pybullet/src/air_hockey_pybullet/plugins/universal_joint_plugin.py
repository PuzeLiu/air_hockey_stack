import numpy as np
import pinocchio as pino


class UniversalJointPlugin:
    def __init__(self, pybullet, namespace, model_spec, **kwargs):
        self.pb = pybullet
        self.namespace = namespace
        self.model_id = model_spec['model_id']

        # Load urdf and pinocchio model
        self.pino_model = pino.buildModelFromUrdf(model_spec['urdf_file'])
        # Add a link of universal joint tip
        se_tip = pino.SE3(np.eye(3), np.array([0., 0., 0.585]))
        self.pino_model.addBodyFrame('striker_rod_tip', 7, se_tip, self.pino_model.nframes - 1)
        self.pino_data = self.pino_model.createData()

        self.joint_indices = np.zeros(self.pino_model.nq)
        self.current_positions = np.zeros(self.pino_model.nq)

        for i, joint in enumerate(model_spec['joint_map'].items()):
            self.joint_indices[i] = joint[1][1]
            self.pb.setJointMotorControl2(*joint[1], controlMode=self.pb.VELOCITY_CONTROL,
                                          force=0.)

    def execute(self, sim_time):
        iiwas_states = np.array(self.pb.getJointStates(self.model_id, self.joint_indices), dtype=object)
        self.current_positions = iiwas_states[:, 0].astype(float)
        self.control_universal_joint()

    def control_universal_joint(self):
        pino.forwardKinematics(self.pino_model, self.pino_data, self.current_positions)
        oMf_tip = pino.updateFramePlacement(self.pino_model, self.pino_data, self.pino_model.nframes - 1)
        oMf_tip.rotation
        q1 = np.arccos(oMf_tip.rotation[:, 2] @ np.array([0., 0., -1.]))
        q2 = 0

        axis = np.cross(oMf_tip.rotation[:, 2], np.array([0., 0., -1.]))
        axis_norm = np.linalg.norm(axis)
        if axis_norm > 1e-2:
            axis = axis / axis_norm
        else:
            axis = np.array([0., 0., 1.])
        q1 = q1 * axis.dot(oMf_tip.rotation[:3, 1])

        self.pb.setJointMotorControlArray(self.model_id, self.joint_indices[7:], self.pb.POSITION_CONTROL,
                                          targetPositions=[q1, q2], positionGains=[0.005, 0.005], velocityGains=[0.001, 0.001])
