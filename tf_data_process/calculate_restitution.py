import os

import numpy as np
import pandas as pd

from utils import read_data_from_dirs


def model(t, param, v0):
    vf = np.exp(- param[0] * t) * v0 + param[1] / param[0] * np.exp(- param[0] * t) - param[1] / param[0]
    return vf


def main():
    data = []
    data.append(read_data_from_dirs(["2020-11-03-14-52-31"]))
    data.append(read_data_from_dirs(["2020-11-03-14-53-29"]))
    data.append(read_data_from_dirs(["2020-11-03-14-54-16"]))
    data.append(read_data_from_dirs(["2020-11-03-14-54-29"]))

    dir = os.path.join(os.path.abspath(__file__ + "../../logs/regression"), "2020-11-08-15-33-06")
    result = pd.read_csv(os.path.join(dir, "result.csv")).to_numpy()
    param = result[0, :2]
    v0_x = result[1]
    v0_y = result[2]
    v0 = np.vstack((v0_x, v0_y)).T

    i = 0
    ratios = []
    frictions = []
    for data_set in data:
        for j in range(len(data_set) - 1):
            if data_set[j + 1][0, -1] - data_set[j][-1, -1] < 0.1:
                t = data_set[j][-1, -1] - data_set[j][0, -1]
                ratio = v0[i + 1] / model(t, param, v0[i])
                if np.any(np.logical_and(ratio < -0.5, ratio > -2)):
                    ratios.extend(ratio[np.logical_and(ratio < 0, ratio > -1)].reshape(-1))
                if np.any(np.logical_and(ratio < 2, ratio > 0.5)):
                    frictions.extend(ratio[np.logical_and(ratio < 10, ratio > 0)].reshape(-1))
            i += 1
    print("ratio: ", np.array(ratios).mean())
    print("friction: ", np.array(frictions).mean())


if __name__ == '__main__':
    main()
