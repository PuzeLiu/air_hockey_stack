import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from glob import glob
from collections import namedtuple

root_dir = os.path.dirname(__file__)
package_dir = os.path.dirname(root_dir)

planners_names_map = dict(ours="ours", sst="SST", mpcmpnet="MPC-MPNet", iros="AQP", nlopt="TrajOpt", cbirrt="CBiRRT")

def mean(r, k, abs=True):
    s = None
    if abs:
        s = [np.abs(v[k]) for _, v in r.items() if k in v and v[k] is not None]
    else:
        s = [v[k] for _, v in r.items() if k in v and v[k] is not None]
    return np.mean(s)

def bar_chart(data, categories):
    plt.rc('font', size=13)
    n_cat = len(list(categories.keys()))
    planners = list(data[list(categories.keys())[0]].keys())

    #planners_results = {}
    #for planner in planners:
    #    results = [data[k][planner] for k, v, in categories.items()]
    #    planners_results[planner] = results
    width = 0.1
    fig, ax = plt.subplots(1, n_cat)
    for i, (k, v) in enumerate(data.items()):
        for j, p in enumerate(planners):
            ax[i].bar(i + width * j, v[p] * categories[k].scale, width, label=planners_names_map[p])
    for i, (k, v) in enumerate(categories.items()):
        if v.log:
            ax[i].set_yscale('log')
        ax[i].set_title(v.title)
        ax[i].set_xticks([])
        ax[i].set_xticks([], minor=True)

    handles, labels = ax[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=n_cat+1, frameon=False)
    plt.show()
    a = 0






def read_results(path):
    results = {}
    for p in glob(os.path.join(path, "*.res")):
        with open(p, 'rb') as fh:
            d = pickle.load(fh)
            name = p.split("/")[-1][:-4]
            xy = name.split("_")
            x = float(xy[0][1:])
            y = float(xy[1][1:])
            results[(x, y)] = d
    return results


results = {}
for path in glob("/home/piotr/b8/ah_ws/results/hitting_exp/*"):
    data = read_results(path)
    name = path.split("/")[-1]
    if name in planners_names_map.keys():
        results[name] = data

description = namedtuple("Description", "title scale log")
categories = {
    "scored": description("Score ratio [%]", 100., False),
    "hit": description("Hit ratio [%]", 100., False),
    "planning_time": description("Mean planning time [ms]", 1., True),
    "puck_velocity_magnitude": description("Mean puck velocity [m/s]", 1., False),
    "actual_hitting_time": description("Mean hitting time [s]", 1., False),
    "actual_z_error": description("Z-axis error [mm⋅s]", 1000., False),
}
summary = {k: {} for k, v in categories.items()}
for name, data in results.items():
    for cat_k, cat_v in categories.items():
        summary[cat_k][name] = mean(data, cat_k)
a = 0

bar_chart(summary, categories)

# box_plot(results, categories)

# box_plot((ours, baseline))
# plot_scatter(ours, "scored")
# plot_scatter(ours, "planned_puck_velocity_magnitude")
# plot_scatter(baseline, "scored")
# plot_scatter(baseline, "planned_puck_velocity_magnitude")
## plot_hist(ours, "puck_actual_vs_planned_velocity_magnitude_error", abs=False, name="ours")
## plot_hist(baseline, "puck_actual_vs_planned_velocity_magnitude_error", abs=False, name="baseline")
# plot_hist(ours, "puck_velocity_magnitude", abs=False, name="ours")
# plot_hist(baseline, "puck_velocity_magnitude", abs=False, name="baseline", alpha=0.5)
# plt.legend()
# plt.show()
## for i in range(6):
##    plt.plot(t, qd_ddot[:, i], label=f"q_ddot_{i}")
## plt.legend()
## plt.show()


# plt.plot(puck[:, 0], puck[:, 1])
# plt.show()
