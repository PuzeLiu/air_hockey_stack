import numpy as np
import os
import matplotlib.pyplot as plt
import rosbag
from kalman_filter.utils import makedirs, plot_trajectory


def read_bad(bag):
    mallet_poses = []
    puck_poses = []
    table_poses = []

    for topic, msg, t in bag.read_messages():
        t_start = bag.get_start_time()
        t_i = t.to_sec() - t_start
        if topic == 'tf':
            if msg.transforms[0].child_frame_id == "Mallet":
                mallet_poses.append([msg.transforms[0].transform.translation.x,
                                     msg.transforms[0].transform.translation.y,
                                     msg.transforms[0].transform.translation.z,
                                     msg.transforms[0].transform.rotation.w,
                                     msg.transforms[0].transform.rotation.x,
                                     msg.transforms[0].transform.rotation.y,
                                     msg.transforms[0].transform.rotation.z,
                                     t_i])
            elif msg.transforms[0].child_frame_id == "Puck":
                puck_poses.append([msg.transforms[0].transform.translation.x,
                                   msg.transforms[0].transform.translation.y,
                                   msg.transforms[0].transform.translation.z,
                                   msg.transforms[0].transform.rotation.w,
                                   msg.transforms[0].transform.rotation.x,
                                   msg.transforms[0].transform.rotation.y,
                                   msg.transforms[0].transform.rotation.z,
                                   t_i])
            elif msg.transforms[0].child_frame_id == "Table":
                table_poses.append([msg.transforms[0].transform.translation.x,
                                    msg.transforms[0].transform.translation.y,
                                    msg.transforms[0].transform.translation.z,
                                    msg.transforms[0].transform.rotation.w,
                                    msg.transforms[0].transform.rotation.x,
                                    msg.transforms[0].transform.rotation.y,
                                    msg.transforms[0].transform.rotation.z,
                                    t_i])

    mallet_poses = np.array(mallet_poses)
    puck_poses = np.array(puck_poses)
    table_poses = np.array(table_poses)

    return mallet_poses, puck_poses, table_poses

def cut_traj(puck_poses, iter, bound_id, save_dir=None):
    traj_part = []

    t = puck_poses[:, -1]
    for t_i in iter:
        data = puck_poses[np.logical_and(np.greater(t, t_i[0]), np.less(t, t_i[1]))]

        if not save_dir is None:
            directory = os.path.join(save_dir, "bound_{}".format(bound_id))
            makedirs(directory)
            id = 0
            while os.path.exists(os.path.join(directory, "data_{}.npy".format(id))):
                id += 1
            np.save(os.path.join(directory, "data_{}.npy".format(id)), data)

        traj_part.append(data)

    return traj_part


if __name__ == "__main__":
    save_dir = os.path.abspath(__file__ + "/../data/rosbag")
    file_name = "2020-10-21-15-31-51"

    bag = rosbag.Bag(os.path.join(save_dir, file_name + ".bag"))

    mallet_poses, puck_poses, table_poses = read_bad(bag)

    plot_trajectory(puck_poses, puck_poses[:, -1], table_poses)

    bound_1_t = [[3.6, 3.8], [4.7, 4.9], [7.5, 7.8]]
    bound_2_t = []
    bound_3_t = [[3.3, 3.4], [4.1, 4.3], [5.5, 5.8]]
    bound_4_t = []

    traj_bound_1 = cut_traj(puck_poses, bound_1_t, 1, save_dir)
    traj_bound_2 = cut_traj(puck_poses, bound_2_t, 2, save_dir)
    traj_bound_3 = cut_traj(puck_poses, bound_3_t, 3, save_dir)
    traj_bound_4 = cut_traj(puck_poses, bound_4_t, 4, save_dir)