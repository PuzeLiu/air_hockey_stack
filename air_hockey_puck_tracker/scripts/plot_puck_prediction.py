
import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt

def com_t(steps):
    return (1/120) * steps * 1e9

def read_puckPose_bag(bag, topicname=''):
    puck_poses = []
    for topic, msg, t in bag.read_messages():
        if topic == '/iiwa_front/marker':
            pose_i = np.array([msg.pose.position.x,
                               msg.pose.position.y,
                               msg.pose.position.z,
                               msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z,
                               msg.pose.orientation.w,
                               t.to_nsec()])
            if msg.pose.position.x + msg.pose.position.y > 0:
                puck_poses.append(pose_i)
        elif topic == '/tf' and msg.transforms[0].child_frame_id == 'Puck':
            pose_i = np.array([msg.transforms[0].transform.translation.x,
                               msg.transforms[0].transform.translation.y,
                               msg.transforms[0].transform.translation.z,
                               msg.transforms[0].transform.rotation.x,
                               msg.transforms[0].transform.rotation.y,
                               msg.transforms[0].transform.rotation.z,
                               msg.transforms[0].transform.rotation.w,
                               t.to_nsec()])
            puck_poses.append(pose_i)
        elif topic == topicname:
            pose_i = np.array([msg.x,
                               msg.y,
                               msg.theta,
                               t.to_nsec()])
            puck_poses.append(pose_i)
    return puck_poses

# data = [(bag_data, predicted_steps, name, topic),...]
def plot_xy_pos_over_t(data):
    fig, (ax1, ax2) = plt.subplots(2, 1)

    ax2.set_ylabel('Y Position')
    ax2.set_xlabel('ROS Time [nsec]')

    ax1.set_ylabel('X Position')
    ax1.set_xlabel('ROS Time [nsec]')
    for d in data:
        poses = read_puckPose_bag(d[0], d[3])
        x = [pos[0] for pos in poses]
        y = [pos[1] for pos in poses]
        t = [pos[-1] + com_t(d[1]) for pos in poses]

        ax1.plot(t, x, label="x_" + str(d[1]) + str(d[2]))
        ax2.plot(t, y, label="y_" + str(d[1]) + str(d[2]))

    ax1.legend()
    ax2.legend()
    plt.show()


def plot_table(data):
    poses = read_puckPose_bag(data)
    x = [pos[0] for pos in poses]
    y = [pos[1] for pos in poses]


if __name__ == "__main__":
    file_name = "Prediction_60_2021-06-25-14-43-04.bag"
    file_path = os.path.join(os.path.abspath(__file__ + "/../data"), file_name)
    prediction60_data = rosbag.Bag(file_path)

    file_name = "prediction_10_2021-06-30-09-17-29.bag"
    file_path = os.path.join(os.path.abspath(__file__ + "/../data"), file_name)
    prediction10_data = rosbag.Bag(file_path)

    file_name = "prediction_240_2021-06-30-09-14-41.bag"
    file_path = os.path.join(os.path.abspath(__file__ + "/../data"), file_name)
    prediction240_data = rosbag.Bag(file_path)

    file_name = "puck2021-06-11-12-02-05.bag"
    file_path = os.path.join(os.path.abspath(__file__ + "/../data"), file_name)
    recorded_data = rosbag.Bag(file_path)

    file_name = "pf_60_2021-06-30-11-39-35.bag"
    file_path = os.path.join(os.path.abspath(__file__ + "/../data"), file_name)
    pf_60_data = rosbag.Bag(file_path)

    file_name = "estimation_2021-07-09-14-35-39.bag"
    file_path = os.path.join(os.path.abspath(__file__ + "/../data"), file_name)
    estimation = rosbag.Bag(file_path)

    file_name = "particleFilter60_2021-07-12-11-34-40.bag"
    file_path = os.path.join(os.path.abspath(__file__ + "/../data"), file_name)
    particle = rosbag.Bag(file_path)

    file_name = "50par60pred_2021-07-12.bag"
    file_path = os.path.join(os.path.abspath(__file__ + "/../data"), file_name)
    pf50pred60 = rosbag.Bag(file_path)

    plot_xy_pos_over_t([(estimation, 0, '', '/iiwa_front/measured'), (estimation, 60, 'pf10', '/iiwa_front/prediction'),
                        (particle, 60, 'kf', '/iiwa_front/prediction'), (pf50pred60, 60, 'pf50', '/iiwa_front/prediction')])

