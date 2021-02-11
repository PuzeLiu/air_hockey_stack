/*
 * MIT License
 * Copyright (c) 2020 Puze Liu, Davide Tateo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "air_hockey_baseline_agent/transformations.h"

using namespace Eigen;
using namespace air_hockey_baseline_agent;

Transformations::Transformations(std::string ns) {
	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_(tfBuffer_);

	std::string ns_prefix;
	if (ns == "/iiwa_front") {
		ns_prefix = 'F';
		tfRobot2Table_ = tfBuffer_.lookupTransform("F_link_0", "TableHome",
				ros::Time(0), ros::Duration(10.0));
		tfRobot2TableInverse_.header = tfRobot2Table_.header;
		tf2::Stamped<tf2::Transform> tmp;
		tf2::fromMsg(tfRobot2Table_, tmp);
		tfRobot2TableInverse_.transform = tf2::toMsg(tmp.inverse());
	} else if (ns == "/iiwa_back") {
		ns_prefix = 'B';
		tfRobot2Table_ = tfBuffer_.lookupTransform("B_link_0", "TableAway",
				ros::Time(0), ros::Duration(10.0));
		tfRobot2TableInverse_.header = tfRobot2Table_.header;
		tf2::Stamped<tf2::Transform> tmp;
		tf2::fromMsg(tfRobot2Table_, tmp);
		tfRobot2TableInverse_.transform = tf2::toMsg(tmp.inverse());
	} else {
		ROS_ERROR_STREAM(
				"Run the node under the namespace: iiwa_front / iiwa_back");
	}

}

void Transformations::transformTrajectory(
		trajectory_msgs::MultiDOFJointTrajectory &cartesianTrajectory) {
	geometry_msgs::Point tmp;
	for (int i = 0; i < cartesianTrajectory.points.size(); ++i) {
		tmp.x = cartesianTrajectory.points[i].transforms[0].translation.x;
		tmp.y = cartesianTrajectory.points[i].transforms[0].translation.y;
		tmp.z = cartesianTrajectory.points[i].transforms[0].translation.z;

		tf2::doTransform(tmp, tmp, tfRobot2Table_);

		cartesianTrajectory.points[i].transforms[0].translation.x = tmp.x;
		cartesianTrajectory.points[i].transforms[0].translation.y = tmp.y;
		cartesianTrajectory.points[i].transforms[0].translation.z = tmp.z;
	}
}

void Transformations::applyForwardTransform(Vector3d &v_in_out) {
	geometry_msgs::Point point;
	point.x = v_in_out.x();
	point.y = v_in_out.y();
	point.z = v_in_out.z();
	tf2::doTransform(point, point, tfRobot2Table_);
	v_in_out.x() = point.x;
	v_in_out.y() = point.y;
	v_in_out.z() = point.z;
}

void Transformations::applyInverseTransform(Vector3d &v_in_out) {
	geometry_msgs::Point point;
	point.x = v_in_out.x();
	point.y = v_in_out.y();
	point.z = v_in_out.z();
	tf2::doTransform(point, point, tfRobot2TableInverse_);
	v_in_out.x() = point.x;
	v_in_out.y() = point.y;
	v_in_out.z() = point.z;
}

void Transformations::applyInverseRotation(Vector3d &v_in_out) {
	geometry_msgs::Vector3 point;
	point.x = v_in_out.x();
	point.y = v_in_out.y();
	point.z = v_in_out.z();

	tf2::doTransform(point, point, tfRobot2TableInverse_);

	v_in_out.x() = point.x;
	v_in_out.y() = point.y;
	v_in_out.z() = point.z;
}

