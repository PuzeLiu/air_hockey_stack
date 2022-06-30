import os.path

import numpy as np
import matplotlib.pyplot as plt
import rosbag

import pinocchio as pino


def read_bag(bag, duration):
    time = []
    joint_state_desired = []
    joint_state_actual = []
    joint_state_error = []
    puck_pose = []

    for topic, msg, t_bag in bag.read_messages():
        if topic == "/iiwa_front/adrc_trajectory_controller/state":
            n_joints = len(msg.joint_names)
            for i in range(n_joints):
                time.append(msg.header.stamp.to_sec())
                joint_state_desired.append(np.concatenate([msg.desired.positions, msg.desired.velocities, msg.desired.accelerations]))
                joint_state_actual.append(np.concatenate([msg.actual.positions, msg.actual.velocities, msg.actual.accelerations]))
                joint_state_error.append(np.concatenate([msg.error.positions, msg.error.velocities, msg.error.accelerations]))
        elif topic == "/tf":
            data = msg.transforms[0]
            frame_id = data.child_frame_id
            if frame_id == "Puck":
                t = data.transform.translation
                dx = 0.19
                dy = 0.813
                dz = 0.03
                puck_pose.append([t.x - dx, t.y - dy, t.z - dz])



    return np.array(time), np.array(joint_state_desired), np.array(joint_state_actual), np.array(joint_state_error),\
           np.array(puck_pose)


root_dir = os.path.dirname(__file__)
package_dir = os.path.dirname(root_dir)
bag_path = os.path.join(package_dir, "test_100hz.bag")
bag_file = rosbag.Bag(bag_path)

robot_file = os.path.join(root_dir, "manifold_planning", "iiwa_striker_new.urdf")
pino_model = pino.buildModelFromUrdf(robot_file)
pino_data = pino_model.createData()
pino_positions = np.zeros(pino_model.nq)

t, desired, actual, error, puck = read_bag(bag_file, 10000)

ee_pos_des = np.zeros((t.shape[0], 3))
ee_pos_actual = np.zeros((t.shape[0], 3))
frame_id = pino_model.getFrameId("F_striker_joint_link")
for i, _ in enumerate(t):
    pino_positions[:7] = desired[i, :7]
    pino.forwardKinematics(pino_model, pino_data, pino_positions)
    pino.updateFramePlacement(pino_model, pino_data, frame_id)
    ee_pos_des[i] = pino_data.oMf[24].translation.copy()

    pino_positions[:7] = actual[i, :7]
    pino.forwardKinematics(pino_model, pino_data, pino_positions)
    pino.updateFramePlacement(pino_model, pino_data, frame_id)
    ee_pos_actual[i] = pino_data.oMf[24].translation.copy()

    J = pino.computeFrameJacobian(pino_model, pino_data, actual[i, :9], frame_id, pino.LOCAL_WORLD_ALIGNED)

plt.plot(ee_pos_des[:, 0], ee_pos_des[:, 1], 'b')
plt.plot(ee_pos_actual[:, 0], ee_pos_actual[:, 1], 'r')
plt.plot(puck[:, 0], puck[:, 1], 'g')
plt.show()


fig, axes = plt.subplots(4, 2)
for i in range(7):
    #axes[int(i/2), i%2].plot(desired[:, i])
    # axes[int(i/2), i%2].plot(actual[:, i])
    axes[int(i/2), i % 2].plot(t, error[:, i])

    axes[int(i/2), i%2].set_title("joint_{}".format(i+1))

fig, axes = plt.subplots(3)
for i in range(3):
    axes[i].plot(t, ee_pos_des[:, i])
    axes[i].plot(t, ee_pos_actual[:, i])

fig, ax = plt.subplots(1)
ax.plot(ee_pos_des[:, 0], ee_pos_des[:, 1])
ax.plot(ee_pos_actual[:, 0], ee_pos_actual[:, 1])

plt.show()
