import numpy as np
import pinocchio as pino


class UniversalJointPlugin:
    def __init__(self, pybullet, namespace, model_spec, **kwargs):
        self.pb = pybullet
        self.namespace = namespace
        self.model_id = model_spec['model_id']

        # Load urdf and pinocchio model
        self.pino_model = pino.buildModelFromUrdf(model_spec['urdf_file'])
        self.pino_data = self.pino_model.createData()

        self.joint_indices = np.zeros(self.pino_model.nq)
        self.current_positions = np.zeros(self.pino_model.nq)

    def execute(self, sim_time):
        iiwas_states = np.array(self.pb.getJointStates(self.model_id, self.joint_indices), dtype=object)
        self.current_positions = iiwas_states[:, 0].astype(float)