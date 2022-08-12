import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from glob import glob
from collections import namedtuple

root_dir = os.path.dirname(__file__)
package_dir = os.path.dirname(root_dir)
baseline_path = os.path.join(package_dir, "results/baseline_opt_lp/")
ours_path = os.path.join(package_dir, "results/ours_opt_lp/")

def make_patch_spines_invisible(ax):
    ax.set_frame_on(True)
    ax.patch.set_visible(False)
    for sp in ax.spines.values():
        sp.set_visible(False)

def box_plot(data):
    extract = lambda x, y: [v[y] for _, v in x.items() if y in v]
    #criteria = ["planned_z_error", "planning_time"]
    #titles = [, "Planning time [ms]"]
    #scales = [(1000., 1000.), (1., 1.)]
    description = namedtuple("Description", "name title scales")
    descriptions = [description("planned_z_error", "Integral of the deviation \n of the plan in z-axis [mm⋅s]", (1000., 1000.)),
                    description("planning_time", "Planning time [ms]", (1., 1.)),
                    description("planned_hitting_time", "Planned hitting \n time [ms]", (1., 1.)),
                    description("planned_puck_velocity_magnitude", "Planned velocity \n magnitude [m/s]", (1., 1.)),
                    description("joint_trajectory_error", "Integral of the joint \n trajectory error [rad⋅s]", (1., 1.)),
                    description("cartesian_trajectory_error", "Integral of the end-effector \n trajectory error [mm⋅s]", (1000., 1000.)),
                    ]
    collection = [[np.array(extract(d, k)) for d in data] for k in [x.name for x in descriptions]]
    #a_planned_z_error = extract(a, "planned_z_error")
    #b_planned_z_error = extract(b, "planned_z_error")
    #data = [a_planned_z_error, b_planned_z_error]
    spacing = 0.2
    c1 = "red"
    c2 = "blue"
    #positions = np.reshape(np.array([[i, i + spacing] for i in range(int(len(data) / 2.))]), -1)

    plt.figure(figsize=(12, 8))
    for i in range(len(descriptions)):
        ax = plt.subplot(1, len(descriptions), 1 + i)
        ax.set_title(descriptions[i].title, rotation=45, ha="left", x=-0.)
        datapoints = [x * descriptions[i].scales[k] for k, x in enumerate(collection[i])]
        bp = ax.boxplot(datapoints, positions=np.arange(0., spacing*len(collection[i]), spacing))
        #bp = ax.boxplot(collection[i], positions=[0, spacing])
        ax.set_xlim(-0.15, 0.35)
        for i in range(len(bp["boxes"])):
            if i % 2:
                bp["boxes"][i].set_color(c1)
                bp["fliers"][i].set_color(c1)
                bp["whiskers"][2 * i].set_color(c1)
                bp["whiskers"][2 * i + 1].set_color(c1)
                bp["caps"][2 * i].set_color(c1)
                bp["caps"][2 * i + 1].set_color(c1)
            else:
                bp["boxes"][i].set_color(c2)
                bp["fliers"][i].set_color(c2)
                bp["whiskers"][2 * i].set_color(c2)
                bp["whiskers"][2 * i + 1].set_color(c2)
                bp["caps"][2 * i].set_color(c2)
                bp["caps"][2 * i + 1].set_color(c2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.set_xticks([])
    plt.subplots_adjust(left=0.1,
                        right=0.9,
                        top=0.7,
                        )
    plt.show()


    #fig, host = plt.subplots()
    #fig.subplots_adjust(right=0.75)

    #par1 = host.twinx()
    #par2 = host.twinx()
    #par2.spines["right"].set_position(("axes", 1.2))
    #make_patch_spines_invisible(par2)
    #par2.spines["right"].set_visible(True)
    ##p1, = host.plot([0, 1, 2], [0, 1, 2], "b-", label="Density")
    #p1 = host.boxplot(data, positions=positions)

    #p2, = par1.plot([0, 1, 2], [0, 3, 2], "r-", label="Temperature")
    #p3, = par2.plot([0, 1, 2], [50, 30, 15], "g-", label="Velocity")
    #host.set_xlim(0, 2)
    #host.set_ylim(0, 2)
    #par1.set_ylim(0, 4)
    #par2.set_ylim(1, 65)

    #host.set_xlabel("Distance")
    #host.set_ylabel("Density")
    #par1.set_ylabel("Temperature")
    #par2.set_ylabel("Velocity")

    #plt.show()

    #plt.boxplot(data, positions=positions)
    #plt.show()

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


def planned_hitting_time(r):
    return mean(r, "planned_hitting_time")


def actual_hitting_time(r):
    return mean(r, "actual_hitting_time")


def joint_trajectory_error(r):
    return mean(r, "joint_trajectory_error")


def puck_velocity(r):
    return mean(r, "puck_velocity_magnitude")


def cartesian_trajectory_error(r):
    return mean(r, "cartesian_trajectory_error")


def mean(r, k, abs=True):
    s = [v[k] for _, v in r.items() if k in v] if not abs else [np.abs(v[k]) for _, v in r.items() if k in v]
    return np.mean(s)


def plot_hist(r, k, abs=True, name="", alpha=1.):
    x = [np.abs(v[k]) for _, v in r.items() if k in v] if abs else [np.abs(v[k]) for _, v in r.items() if k in v]
    print(len(x))
    plt.hist(x, label=name, alpha=alpha, edgecolor='k', bins=20)


def plot_scatter(r, k, abs=True, name="", alpha=1.):
    xy = np.array([v["puck_initial_pose"][:2] for _, v in r.items()])
    v = [np.abs(v[k]) if k in v else 0. for _, v in r.items()] if abs else\
        [v[k] if k in v else 0. for _, v in r.items()]
    plt.scatter(xy[:, 0], xy[:, 1], c=v)
    plt.colorbar()
    #plt.clim(0.5, 2.2)
    plt.xlim(0.7, 1.3)
    plt.ylim(-0.45, 0.45)
    plt.show()


def hit_errors(r):
    magnitude_errors = []
    angle_errors = []
    for k, v in r.items():
        if "puck_actual_vs_planned_velocity_magnitude_error" not in v or "puck_actual_vs_planned_velocity_angle_error" not in v:
            continue
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
#print("VALID RATIOS:")
#print("OURS:", valid_ratio(ours))
#print("BASELINE:", valid_ratio(baseline))
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
print("PLANNED HITIING TIME:")
print("OURS:", planned_hitting_time(ours))
print("BASELINE:", planned_hitting_time(baseline))
print("ACTUAL HITIING TIME:")
print("OURS:", actual_hitting_time(ours))
print("BASELINE:", actual_hitting_time(baseline))
print(ours["K0"])

box_plot((ours, baseline))
plot_scatter(ours, "scored")
plot_scatter(ours, "planned_puck_velocity_magnitude")
plot_scatter(baseline, "scored")
plot_scatter(baseline, "planned_puck_velocity_magnitude")
# plot_hist(ours, "puck_actual_vs_planned_velocity_magnitude_error", abs=False, name="ours")
# plot_hist(baseline, "puck_actual_vs_planned_velocity_magnitude_error", abs=False, name="baseline")
plot_hist(ours, "puck_velocity_magnitude", abs=False, name="ours")
plot_hist(baseline, "puck_velocity_magnitude", abs=False, name="baseline", alpha=0.5)
plt.legend()
plt.show()
# for i in range(6):
#    plt.plot(t, qd_ddot[:, i], label=f"q_ddot_{i}")
# plt.legend()
# plt.show()


# plt.plot(puck[:, 0], puck[:, 1])
# plt.show()
