import numpy as np
import os
import rosbag
import rospy
from iiwas_kinematics_py import Kinematics
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    rospy.init_node('play_bag_data', anonymous=True)

    publisher = rospy.Publisher('/iiwa_front/joint_position_trajectory_controller/command', JointTrajectory, queue_size=1)
    input_dir = os.path.abspath("/home/puze/Desktop")
    file_name = "2021-02-24-17-38-15.bag"

    desire_trajectory = JointTrajectory()
    desire_trajectory.joint_names.append('F_joint_1')
    desire_trajectory.joint_names.append('F_joint_2')
    desire_trajectory.joint_names.append('F_joint_3')
    desire_trajectory.joint_names.append('F_joint_4')
    desire_trajectory.joint_names.append('F_joint_5')
    desire_trajectory.joint_names.append('F_joint_6')
    desire_trajectory.joint_names.append('F_joint_7')

    bag = rosbag.Bag(os.path.join(input_dir, file_name))
    start_point = None
    t_start = None
    for topic, msg, t in bag.read_messages():
        if topic == "/iiwa_front/joint_torque_trajectory_controller/state":
            if start_point == None:
                start_point = msg.actual
                t_start = t
            point_tmp = msg.actual
            if point_tmp.time_from_start < (t-t_start):
                point_tmp.time_from_start = t - t_start
                desire_trajectory.points.append(point_tmp)


    init_traj = JointTrajectory()
    init_traj.joint_names = desire_trajectory.joint_names
    init_traj.points.clear()
    start_point.time_from_start = rospy.Duration(2.0)
    init_traj.points.append(start_point)
    init_traj.header.stamp = rospy.Time.now()
    publisher.publish(init_traj)
    rospy.sleep(2.0)

    desire_trajectory.header.stamp = rospy.Time.now()
    publisher.publish(desire_trajectory)
    rospy.sleep(desire_trajectory.points[-1].time_from_start.to_sec())

if __name__ == '__main__':
    main()