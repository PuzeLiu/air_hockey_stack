import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from glob import glob

root_dir = os.path.dirname(__file__)
package_dir = os.path.dirname(root_dir)
baseline_path = os.path.join(package_dir, "results/baseline_nn/")
ours_path = os.path.join(package_dir, "results/ours_nn/")


def scoring_ratio(r):
    return mean(r, "scored")

def valid_ratio(r):
    return mean(r, "valid")

def planning_time(r):
    return mean(r, "planning_time")


def planned_z_error(r):
    return mean(r, "planned_z_error")


def actual_z_error(r):
    return mean(r, "actual_z_error")


def joint_trajectory_error(r):
    return mean(r, "joint_trajectory_error")


def puck_velocity(r):
    return mean(r, "puck_velocity_magnitude")


def cartesian_trajectory_error(r):
    return mean(r, "cartesian_trajectory_error")


def mean(r, k, abs=True):
    s = 0
    for _, v in r.items():
        if abs:
            s += np.abs(v[k])
        else:
            s += v[k]
    return float(s) / len(r)


def plot_hist(r, k, abs=True, name=""):
    x = [np.abs(v[k]) for _, v in r.items()] if abs else [np.abs(v[k]) for _, v in r.items()]
    plt.hist(x, label=name)


def hit_errors(r):
    magnitude_errors = []
    angle_errors = []
    for k, v in r.items():
        magnitude_errors.append(np.abs(v["puck_actual_vs_planned_velocity_magnitude_error"]))
        angle_errors.append(np.abs(v["puck_actual_vs_planned_velocity_angle_error"]))
    return np.mean(magnitude_errors), np.mean(angle_errors)


def read_results(path):
    results = {}
    for p in glob(path + "*.res"):
        with open(p, 'rb') as fh:
            d = pickle.load(fh)
            results[p[-6:-4]] = d
    return results


ours = read_results(ours_path)
baseline = read_results(baseline_path)
print("SCORING RATIOS:")
print("OURS:", scoring_ratio(ours))
print("BASELINE:", scoring_ratio(baseline))
print("VALID RATIOS:")
print("OURS:", valid_ratio(ours))
print("BASELINE:", valid_ratio(baseline))
print("PLANNING TIMES:")
print("OURS:", 1000 * planning_time(ours))
print("BASELINE:", planning_time(baseline))
print("HIT ERRORS:")
print("OURS:", hit_errors(ours))
print("BASELINE:", hit_errors(baseline))
print("PLANNED Z ERRORS:")
print("OURS:", planned_z_error(ours))
print("BASELINE:", planned_z_error(baseline))
print("ACTUAL Z ERRORS:")
print("OURS:", actual_z_error(ours))
print("BASELINE:", actual_z_error(baseline))
print("JOINT TRAJECTORY ERRORS:")
print("OURS:", joint_trajectory_error(ours))
print("BASELINE:", joint_trajectory_error(baseline))
print("CARTESIAN TRAJECTORY ERRORS:")
print("OURS:", cartesian_trajectory_error(ours))
print("BASELINE:", cartesian_trajectory_error(baseline))
print("MEAN PUCK VELOCITY MAGNITUDE:")
print("OURS:", puck_velocity(ours))
print("BASELINE:", puck_velocity(baseline))
print(ours["K0"])
plot_hist(ours, "puck_actual_vs_planned_velocity_magnitude_error", abs=False, name="ours")
plot_hist(baseline, "puck_actual_vs_planned_velocity_magnitude_error", abs=False, name="baseline")
plt.legend()
plt.show()
# for i in range(6):
#    plt.plot(t, qd_ddot[:, i], label=f"q_ddot_{i}")
# plt.legend()
# plt.show()


# plt.plot(puck[:, 0], puck[:, 1])
# plt.show()
