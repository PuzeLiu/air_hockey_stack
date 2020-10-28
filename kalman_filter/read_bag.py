import os

import matplotlib.pyplot as plt
import numpy as np
import rosbag

from kalman_filter.utils import makedirs, plot_trajectory


def read_bad(bag, bag_new):
    mallet_poses = []
    puck_poses = []
    table_poses = []
    count_table = 0
    count_puck = 0
    count_mallet = 0
    for topic, msg, t in bag.read_messages():

        t_start = bag.get_start_time()
        t_i = t.to_sec() - t_start
        if topic == 'tf':
            if msg.transforms[0].child_frame_id == "Mallet":
                count_mallet += 1
                pose_i = np.array([msg.transforms[0].transform.translation.x,
                                     msg.transforms[0].transform.translation.y,
                                     msg.transforms[0].transform.translation.z,
                                     msg.transforms[0].transform.rotation.w,
                                     msg.transforms[0].transform.rotation.x,
                                     msg.transforms[0].transform.rotation.y,
                                     msg.transforms[0].transform.rotation.z,
                                     t_i])
                if len(mallet_poses) == 0 or not np.equal(np.linalg.norm(mallet_poses[-1][:2] - pose_i[:2]), 0):
                    mallet_poses.append(pose_i)
                    bag_new.write(topic, msg, t)

            elif msg.transforms[0].child_frame_id == "Puck":
                count_puck += 1
                pose_i = np.array([msg.transforms[0].transform.translation.x,
                                   msg.transforms[0].transform.translation.y,
                                   msg.transforms[0].transform.translation.z,
                                   msg.transforms[0].transform.rotation.w,
                                   msg.transforms[0].transform.rotation.x,
                                   msg.transforms[0].transform.rotation.y,
                                   msg.transforms[0].transform.rotation.z,
                                   t_i])
                if len(puck_poses) == 0 or not np.equal(np.linalg.norm(puck_poses[-1][:2] - pose_i[:2]), 0):
                    puck_poses.append(pose_i)
                    bag_new.write(topic, msg, t)

            elif msg.transforms[0].child_frame_id == "Table":
                count_table += 1
                pose_i = np.array([msg.transforms[0].transform.translation.x,
                                   msg.transforms[0].transform.translation.y,
                                   msg.transforms[0].transform.translation.z,
                                   msg.transforms[0].transform.rotation.w,
                                   msg.transforms[0].transform.rotation.x,
                                   msg.transforms[0].transform.rotation.y,
                                   msg.transforms[0].transform.rotation.z,
                                   t_i])
                if len(table_poses) == 0 or not np.equal(np.linalg.norm(table_poses[-1][2] - pose_i[2]), 0):
                    table_poses.append(pose_i)
                    bag_new.write(topic, msg, t)
    print("Found puck TF: {}, used: {}.".format(count_puck, len(puck_poses)))
    print("Found mallet TF: {}, used: {}.".format(count_mallet, len(mallet_poses)))
    print("Found table TF: {}, used: {}.".format(count_table, len(table_poses)))

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


def collision_detection(puck_pose_):
    check_steps = [1, 2, 3, 5]
    max_step = int(np.max(check_steps))
    detection_idx = []
    i = max_step
    while i < puck_pose_.shape[0] - max_step:
        count = 0
        for j in check_steps:
            i_prev = np.clip(i - j, 0, puck_pose_.shape[0] - 1)
            i_post = np.clip(i + j, 0, puck_pose_.shape[0] - 1)
            p_diff_prev = (puck_pose_[i] - puck_pose_[i_prev])[:2]
            p_diff_post = (puck_pose_[i_post] - puck_pose_[i])[:2]
            p_diff_prev_mag = np.linalg.norm(p_diff_prev)
            p_diff_post_mag = np.linalg.norm(p_diff_post)

            # check if start movement
            if p_diff_prev_mag < 1e-3 and p_diff_post_mag > 1e-3:
                count += 1
            elif p_diff_prev_mag * p_diff_post_mag > 1e-5 and \
                    np.dot(p_diff_prev, p_diff_post) / (p_diff_prev_mag * p_diff_post_mag) < 0.99:
                count += 1
        if count >= 3:
            detection_idx.append(i)
        i += 1

    i = 1
    collision_idx = [0]
    point_collect = [detection_idx[0]]
    while i < len(detection_idx):
        if detection_idx[i] - detection_idx[i - 1] == 1:
            point_collect.append(detection_idx[i])
        else:
            collision_idx.append(int(np.mean(point_collect)))
            point_collect = [detection_idx[i]]
        i += 1

    collision_idx.append(puck_pose_.shape[0] - 1)
    return collision_idx


if __name__ == "__main__":
    input_dir = os.path.abspath(__file__ + "/../data/rosbag")
    file_name = "2020-10-22-19-45-13"
    output_dir = os.path.join(input_dir, file_name)

    bag = rosbag.Bag(os.path.join(input_dir, file_name + ".bag"))
    bag_new = rosbag.Bag(os.path.join(input_dir, file_name + "_new.bag"), "w")

    mallet_tf, puck_tf, table_tf = read_bad(bag, bag_new)
    bag_new.close()

    #############################################################
    #                   Cut the trajectory                      #
    #############################################################
    # bound_1_t = [[3.6, 3.8], [4.7, 4.9], [7.5, 7.8]]
    # bound_2_t = []
    # bound_3_t = [[3.3, 3.4], [4.1, 4.3], [5.5, 5.8]]
    # bound_4_t = []
    #
    # traj_bound_1 = cut_traj(puck_poses, bound_1_t, 1, save_dir)
    # traj_bound_2 = cut_traj(puck_poses, bound_2_t, 2, save_dir)
    # traj_bound_3 = cut_traj(puck_poses, bound_3_t, 3, save_dir)
    # traj_bound_4 = cut_traj(puck_poses, bound_4_t, 4, save_dir)

    #############################################################
    #                   Plot velocity profile                   #
    #############################################################

    idx = collision_detection(puck_tf)
    print(idx)
    fig, axes = plt.subplots()
    for i in range(len(idx) - 1):
        axes.plot(puck_tf[idx[i]: idx[i + 1] + 1, 0], puck_tf[idx[i]: idx[i + 1] + 1, 1])
        axes.scatter(puck_tf[idx[i], 0], puck_tf[idx[i], 1], marker=".", s=20)
        axes.text(puck_tf[idx[i], 0], puck_tf[idx[i], 1], str(idx[i]))

    plot_trajectory(puck_tf, puck_tf[:, -1], table_tf, idx)

    plt.show()

    separation_index = [[0, 478, 489, 496, 535, 589],
                        [725, 739, 773, 793, 801, 818, 880, 926],
                        [936, 956, 1135]]

    makedirs(output_dir)
    count = 0
    for separation_set in separation_index:
        for interv_idx in range(len(separation_set) - 1):
            traj_i = puck_tf[separation_set[interv_idx]:separation_set[interv_idx + 1]]
            np.save(os.path.join(output_dir, "{}.npy".format(count)), traj_i)
            count += 1
