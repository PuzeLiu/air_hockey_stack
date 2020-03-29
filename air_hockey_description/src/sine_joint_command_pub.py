#!/usr/bin/python

import rospy
from std_msgs.msg import Float64
import numpy as np

def pub_command():
    topic = '/EffortJointInterface_Position_Controller_B_EB/command'
    topic_config_1 = '/EffortJointInterface_Position_Controller_B_SFE/command'
    topic_config_2 = '/EffortJointInterface_Position_Controller_B_SAA/command'
    topic_config_3 = '/EffortJointInterface_Position_Controller_B_HR/command'
    topic_config_4 = '/EffortJointInterface_Position_Controller_B_WR/command'
    topic_config_5 = '/EffortJointInterface_Position_Controller_B_WAA/command'
    topic_config_6 = '/EffortJointInterface_Position_Controller_B_WFE/command'

    rospy.init_node("cmd_publisher", anonymous=True)
    pub_cmd = rospy.Publisher(topic, Float64, queue_size=10)

    pub_1 = rospy.Publisher(topic_config_1, Float64, queue_size=10)
    pub_2 = rospy.Publisher(topic_config_2, Float64, queue_size=10)
    pub_3 = rospy.Publisher(topic_config_3, Float64, queue_size=10)
    pub_4 = rospy.Publisher(topic_config_4, Float64, queue_size=10)
    pub_5 = rospy.Publisher(topic_config_5, Float64, queue_size=10)
    pub_6 = rospy.Publisher(topic_config_6, Float64, queue_size=10)

    pub_1.publish(0.0)
    pub_2.publish(0.0)
    pub_3.publish(0.0)
    pub_4.publish(0.0)
    pub_5.publish(0.0)
    pub_6.publish(0.0)

    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        cmd = sine(0.1)
        # print(cmd)
        pub_1.publish(np.deg2rad(0.0))
        pub_2.publish(np.deg2rad(0.0))
        pub_3.publish(np.deg2rad(0.0))
        pub_4.publish(np.deg2rad(0.0))
        pub_5.publish(np.deg2rad(0.0))
        pub_6.publish(np.deg2rad(0.0))
        pub_cmd.publish(cmd)
        rate.sleep()

def sine(freq):
    t = rospy.Time().now().to_sec()
    cmd = np.sin(freq* t * 2 * np.pi) * np.pi / 2
    return cmd

def square(freq):
    t = rospy.Time().now().to_sec()
    cmd = np.sign(np.sin(freq* t * 2 * np.pi)) * np.pi / 4 - np.pi/4
    return cmd

if __name__ == '__main__':
    try:
        pub_command()
    except rospy.ROSInterruptException:
        pass