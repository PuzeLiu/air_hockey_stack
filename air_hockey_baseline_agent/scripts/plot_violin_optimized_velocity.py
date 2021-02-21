import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def plot_voilin(data_list, labels):
    ax = plt.subplot(111)
    for i, data in enumerate(data_list):
        ax.violinplot(data[:, -1], points=50, positions=[i],
                      showmeans=True, showextrema=True, quantiles=[0.25, 0.75], widths=0.7)

    ax.set_xticks(np.arange(0, len(labels)))
    ax.set_xticklabels(labels)
    ax.tick_params(axis='both', which='major', labelsize=30)
    plt.show()


def read_no_opt(dir, file_prefix):
    data_middle = pd.read_csv(os.path.join(dir, file_prefix + "_middle.csv"), header=None)
    data_left = pd.read_csv(os.path.join(dir, file_prefix + "_left.csv"), header=None)
    data_right = pd.read_csv(os.path.join(dir, file_prefix + "_right.csv"), header=None)
    data = pd.concat([data_middle, data_left, data_right])
    data = data.to_numpy()
    data = data[data[:, -1] > 0]
    return data


def main():
    dir = "/home/puze/Desktop/100"
    data_no_opt = read_no_opt(dir, 'no_opt')
    data_jac_inv = read_no_opt(dir, 'nl_opt')
    data_lp = read_no_opt(dir, 'lp_nl_opt')
    data_lp_adaptive = read_no_opt(dir, 'weighted_lp_nl_opt')

    # labels = ['no optimization']
    # plot_voilin([data_no_opt], labels)

    labels = ['Heuristics', 'NL Opt', 'NL Opt + LP', 'NL Opt + LP \n + Adaptive Weight']
    plot_voilin([data_no_opt, data_jac_inv, data_lp, data_lp_adaptive], labels)


if __name__ == '__main__':
    main()
