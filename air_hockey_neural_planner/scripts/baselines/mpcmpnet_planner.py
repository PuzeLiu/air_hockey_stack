from time import perf_counter

import numpy as np
from mpcmpnet_params import get_params
from _mpc_mpnet_module import MPCMPNetWrapper


class MPCMPNetPlanner:
    def __init__(self):
        self.params = get_params()


    def solve(self, q0, dq0, qk, dqk):
        start_state = q0.tolist() + dq0.tolist()
        end_state = qk.tolist() + dqk.tolist()
        planner = MPCMPNetWrapper(system_type="iiwa_acc", start_state=start_state, goal_state=end_state,
                                  random_seed=0, goal_radius=self.params['goal_radius'],
                                  sst_delta_near=self.params['sst_delta_near'],
                                  sst_delta_drain=self.params['sst_delta_drain'],
                                  obs_list=np.array([[1., 1.]]), width=self.params['width'], verbose=self.params['verbose'],
                                  mpnet_weight_path=self.params['mpnet_weight_path'],
                                  cost_predictor_weight_path=self.params['cost_predictor_weight_path'],
                                  cost_to_go_predictor_weight_path=self.params[
                                      'cost_to_go_predictor_weight_path'],
                                  num_sample=self.params['cost_samples'],
                                  shm_max_step=self.params['shm_max_steps'],
                                  np=self.params['n_problem'], ns=self.params['n_sample'], nt=self.params['n_t'],
                                  ne=self.params['n_elite'], max_it=self.params['max_it'], converge_r=self.params['converge_r'],
                                  mu_u=self.params['mu_u'], std_u=self.params['sigma_u'], mu_t=self.params['mu_t'],
                                  std_t=self.params['sigma_t'], t_max=self.params['t_max'],
                                  step_size=self.params['step_size'], integration_step=self.params['dt'],
                                  device_id=self.params['device_id'], refine_lr=self.params['refine_lr'],
                                  weights_array=self.params['weights_array'],
                                  obs_voxel_array=np.zeros((10, 10, 3)).reshape(-1)
                                  )
        solution = planner.get_solution()

        tic = perf_counter()
        for iteration in range(int(1e10)):
            planner.mp_path_step(self.params['refine'],
                                 refine_threshold=self.params['refine_threshold'],
                                 using_one_step_cost=self.params['using_one_step_cost'],
                                 cost_reselection=self.params['cost_reselection'],
                                 goal_bias=self.params['goal_bias'],
                                 num_of_problem=self.params['n_problem'])
            solution = planner.get_solution()
            # and np.sum(solution[2]) < th:
            if solution is not None or perf_counter()-tic > self.params['max_planning_time']:
                break
        planning_time = perf_counter() - tic
        q = None
        dq = None
        t = None
        print(solution)
        print(planner.get_approximate_solution())
        solution = solution if solution is not None else planner.get_approximate_solution()
        q = q0[np.newaxis, :6]
        dq = np.zeros_like(q)
        t = np.zeros(1)
        if solution is not None:
            q = solution[0][:, :6]
            dq = solution[0][:, 6:]
            t = np.concatenate([[0.], solution[2]])
            t = np.cumsum(t)
        return q, dq, [], t, planning_time
