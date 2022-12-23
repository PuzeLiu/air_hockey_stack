from time import perf_counter, time

import numpy as np
from mpcmpnet_params import get_params
from _mpc_mpnet_module import MPCMPNetWrapper
from _sst_module import SSTWrapper, IiwaAcc, IiwaAccDistance


class IiwaAcc(IiwaAcc):
    pass

    def distance_computer(self):
        return IiwaAccDistance()


class SSTPlanner:
    def __init__(self):
        self.number_of_iterations = 5000
        self.system = IiwaAcc()
        self.config = dict(
            random_seed=0,
            goal_radius=0.02,
            sst_delta_near=0.05,
            sst_delta_drain=0.02,
            integration_step=0.005,
            min_time_steps=1,
            max_time_steps=20,
            number_of_iterations=300000,
            max_planning_time=60.,
        )


    def solve(self, q0, dq0, qk, dqk):
        start_state = q0[:6].tolist() + dq0[:6].tolist()
        goal_state = qk[:6].tolist() + dqk[:6].tolist()

        planner = SSTWrapper(
            state_bounds=self.system.get_state_bounds(),
            control_bounds=self.system.get_control_bounds(),
            distance=self.system.distance_computer(),
            start_state=start_state,
            goal_state=goal_state,
            goal_radius=self.config['goal_radius'],
            random_seed=0,
            sst_delta_near=self.config['sst_delta_near'],
            sst_delta_drain=self.config['sst_delta_drain']
        )

        tic = perf_counter()
        for iteration in range(self.config["number_of_iterations"]):
            planner.step(self.system, self.config["min_time_steps"], self.config["max_time_steps"],
                         self.config["integration_step"])
            if perf_counter() - tic > self.config["max_planning_time"] or planner.get_solution() is not None:
                break

        planning_time = perf_counter() - tic


        solution = planner.get_solution()
        solution = solution if solution is not None else planner.get_approximate_solution()

        q = solution[0][:, :6]
        dq = solution[0][:, 6:]
        t = np.concatenate([[0.], solution[2]])
        t = np.cumsum(t)
        return q, dq, [], t, planning_time
