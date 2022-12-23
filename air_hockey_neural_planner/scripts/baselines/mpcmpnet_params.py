import numpy as np

def get_params():
    params = {
        'solver_type': "cem",
        'n_problem': 1,
        'n_sample': 64,
        'n_elite': 4,
        'n_t': 1,
        'max_it': 30,
        # 'converge_r': 1e-1,
        'converge_r': 2e-2,

        'dt': 1e-2,
        'mu_u': [0, 0, 0, 0, 0, 0],
        #'sigma_u': [1, 1, 1, 1, 1, 1],
        #'sigma_u': [256.,  256.,  140.8, 140.8,  88.,   32.,   32.],
        #'sigma_u': 0.8 * np.array([256.,  256.,  140.8, 140.8,  88.,   32.]),
        'sigma_u': 0.5 * np.array([256.,  256.,  140.8, 140.8,  88.,   32.]),
        #'sigma_u': 0.1 * np.array([256.,  256.,  140.8, 140.8,  88.,   32.]),
        'mu_t': 1e-1,
        'sigma_t': 0.4,
        't_max': 0.5,
        'verbose': False,  # True, #
        #'step_size': 0.75,
        'step_size': 0.1,

        "goal_radius": 0.02,
        #"sst_delta_near": 0.10,
        #"sst_delta_drain": 0.05,
        "sst_delta_near": 0.05,
        "sst_delta_drain": 0.02,
        "goal_bias": 0.05,

        "width": 6,
        "hybrid": True,
        "hybrid_p": 0.3,
        "cost_samples": 5,
        "mpnet_weight_path": "/home/piotr/b8/new_airhockey/baselines/mpc-mpnet-py/mpnet/output/iiwa_hitting_paper/mpnet_cpu_skip1_p05/ep02600.pth",
        "cost_to_go_predictor_weight_path": "/home/piotr/b8/new_airhockey/baselines/mpc-mpnet-py/mpnet/output/iiwa_hitting_paper/cost_to_go_obs_cpu/ep01700.pth",
        "cost_predictor_weight_path": "",

        "refine": False,
        "using_one_step_cost": False,
        "refine_lr": 0,
        "refine_threshold": 0,
        "device_id": "cpu:0",
        #"device_id": "cuda:3",

        "cost_reselection": False,
        "number_of_iterations": 40,
        #"weights_array": [1, 1, 1., 1.],
        "weights_array": [1, 1, 1, 1, 1, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #"weights_array": [1, 1, 1, 1, 1, 1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3],
        # "weights_array": [],
        #"weights_array": [1, 1, .2, .2],
        'max_planning_time': 60,
        'shm_max_steps': 1
    }

    return params
