/*
 * MIT License
 * Copyright (c) 2020-2021 Puze Liu, Davide Tateo
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

#include "air_hockey_baseline_agent/null_space_optimizer.h"
#include "air_hockey_baseline_agent/hitting_point_optimizer.h"
#include "air_hockey_baseline_agent/planner/combinatorial_hit_new.h"
#include "air_hockey_baseline_agent/planner/cubic_linear_motion.h"
#include "air_hockey_baseline_agent/transformations.h"
#include "air_hockey_baseline_agent/utils.h"
#include <iostream>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <chrono>
#include <cstdio>

using namespace std;
using namespace air_hockey_baseline_agent;
using namespace Eigen;

Transformations setTableRobotTransformation()
{
	// Construct Transform from Table to Robot
	tf2::Stamped<tf2::Transform> tfTable2Robot;
	tfTable2Robot.setOrigin(tf2::Vector3(0.526, 0, 0.1));
	tfTable2Robot.setRotation(tf2::Quaternion(0., 0., 0., 1.));
	geometry_msgs::TransformStamped tfTable2RobotMsg = tf2::toMsg(tfTable2Robot);
	return Transformations(tfTable2RobotMsg);
}

void loadEnvParams(EnvironmentParams& envParams)
{
	envParams.malletRadius = 0.04815;
	envParams.puckRadius = 0.03165;
	envParams.puckHeight = 0.006;
	envParams.tableLength = 1.956;
	envParams.tableWidth = 1.042;
}

void loadAgentParams(AgentParams& agentParams, Transformations transformations)
{
	agentParams.name = "/iiwa_front";
	agentParams.debugTactics = false;
	agentParams.rate = 100.;

	string parent_dir = ros::package::getPath("air_hockey_description");
	string robot_description = parent_dir + "/urdf/iiwa_striker.urdf";
	pinocchio::urdf::buildModel(robot_description, agentParams.pinoModel);
	agentParams.pinoData = pinocchio::Data(agentParams.pinoModel);
	agentParams.pinoFrameId = agentParams.pinoModel.getFrameId("F_striker_tip");
	agentParams.nq = agentParams.pinoModel.nq;

	agentParams.qRef.resize(agentParams.pinoModel.nq);
	//agentParams.qRef.block<7, 1>(0, 0) << 0., 0.06580, 0., -1.45996, 0., 1.22487, 0.;
    agentParams.qRef.block<7, 1>(0, 0) << 0., 0.697, 0, -0.505, 0., 1.92, 0.;
	agentParams.xGoal << 1.98, 0.0;
	agentParams.xHome << 0.08, 0.0, 0.0;
	agentParams.xInit << 0.4, 0.0, 0.0;

	agentParams.hitRange << 0.2, 0.8;

	agentParams.staticVelocityThreshold = 0.08;
	agentParams.eeMaxAcceleration = 6.;
	agentParams.defendMinTime = 0.3;
	agentParams.defendZoneWidth = 0.4;
	agentParams.defendLine = 0.2;
	agentParams.planTimeOffset = 0.1;
	agentParams.tPredictionMax = 1.5;
	agentParams.tTacticSwitchMin = 1.5;
	agentParams.hitVelocityScale = 1.0;
	agentParams.initHeight = 0.2;
	agentParams.universalJointHeight = 0.06;  // Table Height: 0.0645 + 0.1 but 0.0045 will be handled by compliance
	pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, agentParams.qRef);
	pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);

	// Construct qHome
	Vector3d xTmp;
	xTmp << agentParams.xHome[0], agentParams.xHome[1], agentParams.universalJointHeight;
	transformations.transformTable2Robot(xTmp);
	pinocchio::SE3 oMhome(agentParams.pinoData.oMf[agentParams.pinoFrameId].rotation(), xTmp);
	air_hockey_baseline_agent::inverseKinematics(agentParams, oMhome, agentParams.qRef, agentParams.qHome);
	// Construct qInit
	xTmp << agentParams.xHome[0], agentParams.xHome[1], agentParams.universalJointHeight + agentParams.initHeight;
	transformations.transformTable2Robot(xTmp);
	pinocchio::SE3 oMinit(agentParams.pinoData.oMf[agentParams.pinoFrameId].rotation(), xTmp);
	air_hockey_baseline_agent::inverseKinematics(agentParams, oMinit, agentParams.qRef, agentParams.qInit);
}


void initOptimizerData(OptimizerData& optData)
{
	optData.K = 100.;
	optData.weights << 10., 10., 5., 10., 1., 1., 0.;
	optData.weightsAnchor << 1., 1., 5., 1, 10., 10., 0.;
}

void getStopPoint(const Vector2d &hitPoint, const AgentParams &agentParams,
	const EnvironmentParams &envParams, Vector2d &stopPoint) {
	if (hitPoint.y() > 0) {
		stopPoint = hitPoint + Vector2d(0.2, -0.25);
	} else {
		stopPoint = hitPoint + Vector2d(0.2, 0.25);
	}

	stopPoint.x() = fmin(stopPoint.x(), agentParams.hitRange[1]);
	stopPoint.y() = fmax(fmin(stopPoint.y(), envParams.tableWidth / 2 - envParams.malletRadius - 0.05),
		-envParams.tableWidth / 2 + envParams.malletRadius + 0.05);
}

inline string leading_zeros(int n, int len)
{
    string result(len--, '0');
    for (int val=(n<0)?-n:n; len>=0&&val!=0; --len,val/=10)
        result[len]='0'+val%10;
    if (len>=0&&n<0) result[0]='-';
    return result;
}

int main(int argc, char* argv[])
{
	ros::Time::init();
	rosbag::Bag bag;
	std::string log_path = ros::package::getPath("air_hockey_baseline_agent");

	srand((unsigned int) time(0));

	EnvironmentParams envParams{};
	loadEnvParams(envParams);

	Transformations transformations = setTableRobotTransformation();

	AgentParams agentParams;
	loadAgentParams(agentParams, transformations);

	OptimizerData optData(agentParams);
	initOptimizerData(optData);

	NullSpaceOptimizer nullOptimizer(agentParams, optData);
	HittingPointOptimizer hittingPointOptimizer(agentParams, optData);

	// Construct Planner
	Vector2d bound_lower(envParams.malletRadius + 0.02,
		-envParams.tableWidth / 2 + envParams.malletRadius + 0.02);
	Vector2d bound_upper(envParams.tableLength / 2 - envParams.malletRadius,
		envParams.tableWidth / 2 - envParams.malletRadius - 0.02);
	CombinatorialHitNew combPlanner(bound_lower, bound_upper, agentParams.rate,
		agentParams.universalJointHeight, agentParams.eeMaxAcceleration);
	CubicLinearMotion cubPlanner(agentParams.rate, agentParams.universalJointHeight);

	trajectory_msgs::MultiDOFJointTrajectory cartTraj;
	cartTraj.joint_names.push_back("x");
	cartTraj.joint_names.push_back("y");
	cartTraj.joint_names.push_back("z");
	trajectory_msgs::JointTrajectory jointTraj;
	jointTraj.joint_names.push_back("F_joint_1");
	jointTraj.joint_names.push_back("F_joint_2");
	jointTraj.joint_names.push_back("F_joint_3");
	jointTraj.joint_names.push_back("F_joint_4");
	jointTraj.joint_names.push_back("F_joint_5");
	jointTraj.joint_names.push_back("F_joint_6");
	jointTraj.joint_names.push_back("F_joint_7");

	bool success;

	vector<vector<float>> data;
	vector<int> successes = {};
	vector<float> times = {};
    string parent_dir = ros::package::getPath("air_hockey_baseline_agent");
    string ds = "val";
    string ds_name = "airhockey_table_moves_v08_a10v_optimized_hitting_regularized_man_lp_new";
    //string data_file = parent_dir + "/data/airhockey_maximal_velocity_hitting/" + ds + "/data.tsv";
    string data_file = parent_dir + "/data/" + ds_name + "/" + ds + "/data.tsv";
    ifstream fin(data_file);
    string line;
    while (getline(fin, line)) {
        // Split line into tab-separated parts
        vector<string> parts;
        split(parts, line, boost::is_any_of("\t"));
        vector<float> record(parts.size());
        transform(parts.begin(), parts.end(), record.begin(),
                  [](string const& val) {return stof(val);});

        cout << "First of " << parts.size() << " elements: " << parts[0] << endl;
        data.push_back(record);
    }
    fin.close();
    //for (auto r: data) {
    //    for (auto x: r) {
    //        cout << x << " ";
    //    }
    //    cout << endl;
    //}

	// Setup starting and stop point on the table surface
	Vector2d xStart2d, vStart2d, xHit2d, vHit2d, xEnd2d, vEnd2d, xStop2d, vStop2d;
    vStart2d.setZero();
    for (int k=0; k < data.size(); k++) {
    //for (int k=14623; k < data.size(); k++) {
	//for (int k=0; k < 500; k++) {
	    // random settings
        //xStart2d << agentParams.xHome[0], agentParams.xHome[1]; // The start point and qStart in nullOptimizer should match
        //xHit2d.setRandom();
        //xHit2d[0] = xHit2d[0] * (agentParams.hitRange[1] - agentParams.hitRange[0]) / 2
        //            + (agentParams.hitRange[0] + agentParams.hitRange[1]) / 2;
        //xHit2d[1] = xHit2d[1] * (envParams.tableWidth / 2 - 0.12 - envParams.malletRadius);
        //vHit2d.setRandom();
        //vHit2d[0] = (vHit2d[0] + 1) / 2;
        //vHit2d[1] = vHit2d[1];
        //getStopPoint(xHit2d, agentParams, envParams, xStop2d);
        //vStop2d.setZero();

        // dataset settings
        vector<float> record = data[k];
        xStart2d << record[38] - 0.526, record[39];
        agentParams.qInit.block<7, 1>(0, 0) << record[0], record[1], record[2], record[3], record[4], record[5], record[6];
        agentParams.qHome.block<7, 1>(0, 0) << record[0], record[1], record[2], record[3], record[4], record[5], record[6];

        pinocchio::forwardKinematics(agentParams.pinoModel, agentParams.pinoData, agentParams.qInit);
        pinocchio::updateFramePlacements(agentParams.pinoModel, agentParams.pinoData);
        auto xStart3d = agentParams.pinoData.oMf[agentParams.pinoFrameId].translation();
        xStart2d << xStart3d[0] - 0.526, xStart3d[1];
        //v0 = record[40];
        //vStart2d << v0 * cos(record[39]), v0 * sin(record[39]);
        xHit2d << record[14] - 0.526, record[15];
        vHit2d << cos(record[16]), sin(record[16]);

        //cout << "NEW:" << endl;
        //cout << "XSTART: " << xStart2d << endl;
        //cout << "XHIT: " << xHit2d << endl;
        //cout << "VHIT: " << vHit2d << endl;

        //! Get Maximum Hitting Velocity
        double velMag, hitting_time;
        VectorXd qAnchor(agentParams.nq);
        VectorXd dqStart(agentParams.nq);
        qAnchor.setZero();
        Vector3d hitPoint3d, hitDir3d;
        hitPoint3d.topRows(2) = xHit2d;
        hitPoint3d[2] = agentParams.universalJointHeight;
        hitDir3d.topRows(2) = vHit2d;
        hitDir3d[2] = 0.;
        hitDir3d.normalize();
        transformations.transformTable2Robot(hitPoint3d);
        transformations.rotationTable2Robot(hitDir3d);

        hittingPointOptimizer.solve(hitPoint3d, hitDir3d, qAnchor, velMag);
        hitDir3d = hitDir3d * velMag;
        vHit2d = hitDir3d.topRows(2);

        /** AQP Optimization*/
        cout << "#################################" << endl;
        cout << "#    Test AQP Optimization      #" << endl;
        cout << "#################################" << endl;
        success = false;
        auto start = chrono::high_resolution_clock::now();
        for (int j = 0; j < 20; ++j) {
            cartTraj.points.clear();
            jointTraj.points.clear();

            if (not combPlanner.planHit(xStart2d, vStart2d, xHit2d, vHit2d, hitting_time, xStop2d, vStop2d, cartTraj)) {
                break;
            }
            transformations.transformTrajectory(cartTraj);

            dqStart.setZero();
            if (nullOptimizer.optimizeJointTrajectoryAnchor(cartTraj, agentParams.qHome, dqStart, qAnchor, hitting_time, jointTraj)) {
                success = true;
                break;
            } else {
                vHit2d = vHit2d * 0.9;
                continue;
            }
        }
        auto finish = chrono::high_resolution_clock::now();
        cout << "Number of Points: " << cartTraj.points.size() << ", Success: " << success << ", Optimization Time: "
             << chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() / 1.e6 << "ms\n";

        successes.push_back((int)success);
        times.push_back(chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() / 1.e6);

        //for (auto p: jointTraj.points) {
        //    for (auto x: p.positions)
        //        cout << x << "\t";
        //    for (auto v: p.velocities)
        //        cout << v << "\t";
        //    cout << endl;
        //}
        auto fileName = "./data/" + ds_name + "/" + ds + "/" + leading_zeros(k, 5) + ".path";
        FILE* fout = fopen(fileName.c_str(), "w");
        for (int j = 0; j < jointTraj.points.size(); j++)
        {
            for (int i = 0; i < 7; i++)
                fprintf(fout, "%.5f\t", jointTraj.points[j].positions[i]);
            for (int i = 0; i < 6; i++)
                fprintf(fout, "%.5f\t", jointTraj.points[j].velocities[i]);
            fprintf(fout, "%.5f\n", jointTraj.points[j].velocities[6]);
        }
        fclose(fout);

        //bag.open(log_path + "/examples/aqp_opt.bag", rosbag::bagmode::Write);
        //bag.write("cartesian_trajectory", ros::Time::now(), cartTraj);
        //bag.write("joint_trajectory", ros::Time::now(), jointTraj);
        //bag.close();
    }
    auto fileName = "./data/" + ds + "/opt_stats.tsv";
    FILE* fout = fopen(fileName.c_str(), "w");
    for (int j = 0; j < data.size(); j++)
    {
        fprintf(fout, "%d\t", successes[j]);
        fprintf(fout, "%.5f\n", times[j]);
    }
    fclose(fout);

	return 0;
}
