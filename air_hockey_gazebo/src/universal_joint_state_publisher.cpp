#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <boost/algorithm/clamp.hpp>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

int main(int argc, char **argv) {
    ros::init(argc, argv, "universal_joint_state_publisher");
    ros::NodeHandle nh("/");
    ros::NodeHandle nh_p("~");
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener(buffer);
    ros::Publisher jointStatePub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);


    double frequency;
    nh_p.getParam("frequency", frequency);
    ros::Rate rate(frequency);
    std::string baseName, eeName, jointPrefix;


    if (nh.getNamespace() == "/iiwa_front") {
        jointPrefix = 'F';
    } else if (nh.getNamespace() == "/iiwa_back") {
        jointPrefix = 'B';
    } else {
        ROS_ERROR_STREAM("Wrong Namespace");
        return -1;
    }

    baseName = jointPrefix + "_link_0";
    eeName = jointPrefix + "_link_ee";

    geometry_msgs::TransformStamped tfEE;

    double q1, q2;
    sensor_msgs::JointState universalJointState;
    universalJointState.name.push_back(jointPrefix + "_striker_joint_1");
    universalJointState.name.push_back(jointPrefix + "_striker_joint_2");
    universalJointState.position.resize(2);
    universalJointState.velocity.push_back(0.);
    universalJointState.velocity.push_back(0.);
    universalJointState.effort.push_back(0.);
    universalJointState.effort.push_back(0.);

    ros::Duration(1.0).sleep();
    while (ros::ok()) {
        try {
            tfEE = buffer.lookupTransform(baseName, eeName, ros::Time(0), ros::Duration(1.0));
            Eigen::Quaterniond quat(tfEE.transform.rotation.w,
                                    tfEE.transform.rotation.x,
                                    tfEE.transform.rotation.y,
                                    tfEE.transform.rotation.z);

//            if (pow(quat.w, 2) - pow(quat.x, 2) - pow(quat.y, 2) + pow(quat.z, 2) != 0){
//                q1 = atan(-2 * (quat.w * quat.y - quat.x * quat.z) /
//                          (pow(quat.w, 2) - pow(quat.x, 2) - pow(quat.y, 2) + pow(quat.z, 2)));
//            } else {
//                q1 = M_PI_2;
//            }
//            q2 = asin(boost::algorithm::clamp((quat.w * quat.x - quat.y, quat.z) * 2, -1, 1));

            q1 = acos(quat.toRotationMatrix().col(2).dot(Eigen::Vector3d(0., 0., -1)));
            q2 = 0.;

            universalJointState.position[0] = q1;
            universalJointState.position[1] = q2;
            universalJointState.header.stamp = ros::Time::now();
            jointStatePub.publish(universalJointState);

        } catch (tf2::TransformException &ex) {
//            ROS_ERROR_STREAM(ex.what());
        }
        rate.sleep();
    }
    return 0;
}
