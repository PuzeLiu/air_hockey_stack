from copy import copy
from time import perf_counter

import numpy as np
import pinocchio as pino
from ompl import base as ob
from ompl import geometric as og

from manifold_planning.utils.constants import Limits


class TableConstraint(ob.Constraint):

    def __init__(self, pino_model, pino_data, joint_id):
        super(TableConstraint, self).__init__(6, 1)
        self.pino_model = pino_model
        self.pino_data = pino_data
        self.joint_id = joint_id

    def function(self, x, out):
        q = np.concatenate([x, np.zeros(3)], axis=-1)
        pino.forwardKinematics(self.pino_model, self.pino_data, q)
        xyz_pino = self.pino_data.oMi[-1].translation
        out[:] = xyz_pino[-1] - 0.16

    def jacobian(self, x, out):
        q = np.concatenate([x, np.zeros(3)], axis=-1)
        J_z = pino.computeFrameJacobian(self.pino_model, self.pino_data, q, self.joint_id, pino.LOCAL_WORLD_ALIGNED)[2:3, :6]
        out[:] = J_z[0]


class CBiRRTPlanner:
    def __init__(self, n_pts, pino_model, joint_id):
        self.N = n_pts
        self.M = self.N - 2
        self.D = 6
        self.pino_model = pino_model
        self.pino_data = self.pino_model.createData()
        self.joint_id = joint_id

        self.time = 60.
        tolerance = 1e-2#ob.CONSTRAINT_PROJECTION_TOLERANCE
        tries = ob.CONSTRAINT_PROJECTION_MAX_ITERATIONS
        lambda_ = ob.CONSTRAINED_STATE_SPACE_LAMBDA
        delta = ob.CONSTRAINED_STATE_SPACE_DELTA

        rvss = ob.RealVectorStateSpace(6)
        bounds = ob.RealVectorBounds(6)
        lb = pino_model.lowerPositionLimit[:6]
        ub = pino_model.upperPositionLimit[:6]
        for i in range(6):
            bounds.setLow(i, lb[i])
            bounds.setHigh(i, ub[i])
        rvss.setBounds(bounds)

        # Create our constraint.
        constraint = TableConstraint(self.pino_model, self.pino_data, self.joint_id)
        constraint.setTolerance(tolerance)
        constraint.setMaxIterations(tries)
        self.css = ob.ProjectedStateSpace(rvss, constraint)
        self.csi = ob.ConstrainedSpaceInformation(self.css)
        self.ss = og.SimpleSetup(self.csi)

        self.css.setDelta(delta)
        self.css.setLambda(lambda_)

        self.planner = og.RRTConnect(self.csi)
        self.ss.setPlanner(self.planner)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.obstacles))

        self.css.setup()
        self.ss.setup()


    def obstacles(self, x):
        q = np.zeros((9))
        for i in range(6):
            q[i] = x[i]
        pino.forwardKinematics(self.pino_model, self.pino_data, q)
        xyz = self.pino_data.oMi[-1].translation
        if xyz[0] < 0.6 or xyz[0] > 2.:
            return False
        if xyz[1] < -0.45 or xyz[1] > 0.45:
            return False
        return True

    def solve(self, q0, dq0, qk, dqk):
        self.ss.clear()
        start = ob.State(self.css)
        goal = ob.State(self.css)
        for i in range(6):
            start[i] = q0[i]
        for i in range(6):
            goal[i] = qk[i]
        self.ss.setStartAndGoalStates(start, goal, 0.01)
        stat = self.ss.solve(self.time)
        planning_time = self.ss.getLastPlanComputationTime()
        success = False
        q = None
        dq = None
        ddq = None
        t = None
        if stat:
            # Get solution and validate
            path = self.ss.getSolutionPath()
            path.interpolate()
            states = [[x[i] for i in range(6)] for x in path.getStates()]
            q = np.array(states)
            dq = np.zeros_like(q)
            ddq = np.zeros_like(q)
            success = True
            q_diff = q[1:] - q[:-1]
            diff = np.sum(np.abs(q_diff), axis=-1)
            include = np.concatenate([diff > 0, [True]])
            q = q[include]
            q_diff = q_diff[include[:-1]]
            ts = np.abs(q_diff) / Limits.q_dot
            t = np.max(ts, axis=-1)
            t = np.concatenate([[0.], t + 1e-4])
            t = np.cumsum(t)
        return q, [], [], t, planning_time
