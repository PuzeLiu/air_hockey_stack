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


#include <ros/package.h>
#include "air_hockey_puck_tracker/Validation.hpp"

using namespace std;

namespace air_hockey_baseline_agent {

ValidationInterface::ValidationInterface(ros::NodeHandle nh_, bool save) :
		m_nh(nh_) {
	m_pub_predict = m_nh.advertise < air_hockey_puck_tracker::InnovationMsg
			> ("prediction", 1000);
	m_pub_true = m_nh.advertise < air_hockey_puck_tracker::InnovationMsg
			> ("measured", 1000);
	m_pub_diff = m_nh.advertise < air_hockey_puck_tracker::InnovationMsg
			> ("difference", 1000);
	m_save = save;
	if (m_save) {
		string path = ros::package::getPath("air_hockey_puck_tracker");
		stringstream filepath;
		auto t = std::time(nullptr);
		auto tm = *std::localtime(&t);
		filepath << path << "/data/" << std::put_time(&tm, "%Y-%d-%m-%H-%M-%S")
				<< ".bag";

		bag.open(filepath.str(), rosbag::bagmode::Write);
	}

}

ValidationInterface::~ValidationInterface() {
	bag.close();
}

void ValidationInterface::record(const Measurement &prediction,
		const Measurement &measurement, const InnovationCovariance &S) {
	m_msg.header.stamp = ros::Time::now();
	m_msg.size = 1;
	m_msg.x = prediction.x();
	m_msg.y = prediction.y();
	m_msg.theta = prediction.theta();
	m_msg.normalized = prediction.transpose() * S.inverse() * prediction;
	m_pub_predict.publish(m_msg);

	m_msg.size = 1;
	m_msg.x = measurement.x();
	m_msg.y = measurement.y();
	m_msg.theta = measurement.theta();
	m_msg.normalized = measurement.transpose() * S.inverse() * measurement;
	m_pub_true.publish(m_msg);

	m_msg.size = 1;
	m_msg.x = prediction.x() - measurement.x();
	m_msg.y = prediction.y() - measurement.y();
	m_msg.theta = prediction.theta() - measurement.theta();
	m_msg.normalized = measurement.transpose() * S.inverse() * measurement;
	m_pub_diff.publish(m_msg);

	if (m_save) {
		bag.write("innovation", ros::Time::now(), m_msg);
	}
}

}

