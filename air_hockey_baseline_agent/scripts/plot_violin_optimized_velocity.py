import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy import stats


def plot_voilin(data_list, out_dir):
    ax = plt.subplot(111)
    ax.set_position([0.1, 0.25, 0.8, 0.7])
    position = 0
    position_list = []
    label_list = []
    color_dict = {'no_opt': 'tab:blue', 'nl_opt': 'tab:orange', 'lp_nl_opt': 'tab:green'}
    for data_dict in data_list:
        for alg in data_dict:
            position += 1
            data = data_dict[alg][:, -2]
            # data = data[np.where(data != 0)]
            pc = ax.violinplot(data, [position], showmeans=True,
                               showmedians=True, quantiles=[0.25, 0.75], points=100, widths=0.75)
            pc['bodies'][0].set_color(color_dict[alg])
            pc['cmeans'].set_color(color_dict[alg])
            pc['cmeans'].set_lw(3)
            pc['cmins'].set_color(color_dict[alg])
            pc['cmaxes'].set_color(color_dict[alg])
            pc['cbars'].set_color(color_dict[alg])
            pc['cmedians'].set_color(color_dict[alg])
            pc['cmedians'].set_lw(1)
            pc['cmedians'].set_ls(':')
            pc['cquantiles'].set_color(color_dict[alg])
            pc['cquantiles'].set_lw(1)
            pc['cquantiles'].set_ls('-.')
        position += 2

    ax.fill_between([], [], color=color_dict['no_opt'], label='QP')
    ax.fill_between([], [], color=color_dict['nl_opt'], label='NLP+AQP')
    ax.fill_between([], [], color=color_dict['lp_nl_opt'], label='LP+NLP+AQP')

    ax.set_xticks([2, 7, 12])
    ax.set_xticklabels(['Forward Bounce', 'Direct Hit', 'Reverse Bounce'], fontsize=15)
    ax.legend(bbox_to_anchor=(0.5, -0.1), loc='upper center', ncol=3, fontsize=15)
    plt.savefig(os.path.join(out_dir, 'with_failure.pdf'))
    plt.show()


def read_no_opt(dir, file_prefix):
    data_no_opt = pd.read_csv(os.path.join(dir, file_prefix, "no_opt.csv"), header=None)
    data_nl_opt = pd.read_csv(os.path.join(dir, file_prefix, "nlopt.csv"), header=None)
    data_lp_nl_opt = pd.read_csv(os.path.join(dir, file_prefix, "lp_nlopt.csv"), header=None)
    data_dict = {'no_opt': data_no_opt.to_numpy(),
                 'nl_opt': data_nl_opt.to_numpy(),
                 'lp_nl_opt': data_lp_nl_opt.to_numpy()}
    return data_dict


def get_optimization_time(data_list):
    opt_time = {'nl_opt': np.array([]), 'lp_nl_opt': np.array([]), 'no_opt': np.array([])}
    for data_dict in data_list:
        for alg in data_dict:
            opt_time[alg] = np.concatenate([opt_time[alg], data_dict[alg][:, -1]])

    print('no_opt: ')
    print("mean: {}, min: {}, max: {}, median: {}, quantile: {}".format(np.mean(opt_time['no_opt']),
                                                                        np.min(opt_time['no_opt']),
                                                                        np.max(opt_time['no_opt']),
                                                                        np.median(opt_time['no_opt']),
                                                                        np.quantile(opt_time['no_opt'],
                                                                                    [0.25, 0.75])))
    print('nl_opt: ')
    print("mean: {}, min: {}, max: {}, median: {}, quantile: {}".format(np.mean(opt_time['nl_opt']),
                                                                        np.min(opt_time['nl_opt']),
                                                                        np.max(opt_time['nl_opt']),
                                                                        np.median(opt_time['nl_opt']),
                                                                        np.quantile(opt_time['nl_opt'],
                                                                                    [0.25, 0.75])))
    print('lp_nl_opt: ')
    print("mean: {}, min: {}, max: {}, median: {}, quantile: {}".format(np.mean(opt_time['lp_nl_opt']),
                                                                        np.min(opt_time['lp_nl_opt']),
                                                                        np.max(opt_time['lp_nl_opt']),
                                                                        np.median(opt_time['lp_nl_opt']),
                                                                        np.quantile(opt_time['lp_nl_opt'],
                                                                                    [0.25, 0.75])))


def main():
    data_dir = "/home/puze/Dropbox/PHD/AirHockey/IROS/hitting_point_optimization/3_levels"
    data_easy = read_no_opt(data_dir, 'easy')
    data_medium = read_no_opt(data_dir, 'medium')
    data_hard = read_no_opt(data_dir, 'hard')

    # labels = ['no optimization']
    # plot_voilin([data_no_opt], labels)
    plot_voilin([data_easy, data_medium, data_hard], data_dir)
    get_optimization_time([data_easy, data_medium, data_hard])


if __name__ == '__main__':
    main()
