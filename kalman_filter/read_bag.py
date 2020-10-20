import numpy as np
import os
import matplotlib.pyplot as plt
import rosbag
from kalman_filter.utils import makedirs, plot_trajectory

if __name__ == "__main__":
    directory = os.path.abspath(__file__ + "/../data/rosbag")
    file_name = "2020-10-20-17-01-27"

    bag = rosbag.Bag(os.path.join(directory, file_name + ".bag"))

    t_start = bag.get_start_time()

    mallet_poses = []
    puck_poses = []
    table_poses = []

    for topic, msg, t in bag.read_messages():
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
                mallet_poses.append([msg.transforms[0].transform.translation.x,
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

    output_dir = os.path.join(directory, file_name)
    makedirs(output_dir)

    np.save(os.path.join(output_dir, "mallet.npy"), mallet_poses)
    np.save(os.path.join(output_dir, "puck.npy"), puck_poses)
    np.save(os.path.join(output_dir, "table.npy"), table_poses)

    plot_trajectory(puck_poses, puck_poses[:, -1])
