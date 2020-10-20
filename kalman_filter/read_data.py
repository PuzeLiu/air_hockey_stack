import numpy as np
import os

if __name__ == "__main__":
    data_count = 0
    directory = os.path.abspath(__file__ + "/../data")
    file_name = "data_{}.npy".format(data_count)

    data = np.load(os.path.join(directory, file_name))
    print(data.shape)