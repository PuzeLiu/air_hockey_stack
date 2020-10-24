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


#ifndef PUCK_TRACKER_VISUALIZATION_INTERFACE_H
#define PUCK_TRACKER_VISUALIZATION_INTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <mrpt/bayes/CKalmanFilterCapable.h>

class VisualizationInterface{
public:
    ros::NodeHandle m_nh;

    ros::Publisher m_tablePub;
    ros::Publisher m_puckPub;
    ros::Publisher m_malletPub;
    ros::Publisher m_predictionPub;

    visualization_msgs::Marker m_tableMarker;
    visualization_msgs::Marker m_puckMarker;
    visualization_msgs::Marker m_malletMarker;
    visualization_msgs::Marker m_predictionMarker;

    tf::TransformListener m_listener;

    tf::StampedTransform m_tfTable, m_tfPuck, m_tfMallet;

    VisualizationInterface(const ros::NodeHandle& nh) : m_nh(nh){
        m_tablePub = m_nh.advertise<visualization_msgs::Marker>("marker_table", 1);
        m_puckPub = m_nh.advertise<visualization_msgs::Marker>("marker_puck", 1);
        m_malletPub = m_nh.advertise<visualization_msgs::Marker>("marker_mallet", 1);
        m_predictionPub = m_nh.advertise<visualization_msgs::Marker>("marker_prediction", 1);

        m_tableMarker.header.frame_id = "world";
        m_tableMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
        m_tableMarker.action = visualization_msgs::Marker::ADD;
        m_tableMarker.scale.x = 1.0;
        m_tableMarker.scale.y = 1.0;
        m_tableMarker.scale.z = 1.0;
        m_tableMarker.mesh_resource = "package://air_hockey_description/world/models/air_hockey_table/mesh/table.dae";
        m_tableMarker.mesh_use_embedded_materials = true;
        m_tableMarker.color.r = 0.9;
        m_tableMarker.color.g = 0.9;
        m_tableMarker.color.b = 0.9;
        m_tableMarker.color.a = 1.0;
        m_tableMarker.pose.position.x = 0.;
        m_tableMarker.pose.position.y = 0.;
        m_tableMarker.pose.position.z = 0.;
        m_tableMarker.pose.orientation.w = 1.;
        m_tableMarker.pose.orientation.x = 0.;
        m_tableMarker.pose.orientation.y = 0.;
        m_tableMarker.pose.orientation.z = 0.;

        m_puckMarker.header.frame_id = "world";
//        m_puckMarker.header.stamp = ros::Time::now();
        m_puckMarker.type = visualization_msgs::Marker::CYLINDER;
        m_puckMarker.action = visualization_msgs::Marker::ADD;
        m_puckMarker.scale.x = 0.0633;
        m_puckMarker.scale.y = 0.0633;
        m_puckMarker.scale.z = 0.005;
        m_puckMarker.color.r = 1.0;
        m_puckMarker.color.g = 0.0;
        m_puckMarker.color.b = 0.0;
        m_puckMarker.color.a = 1.0;
        m_puckMarker.pose.position.x = 0.;
        m_puckMarker.pose.position.y = 0.;
        m_puckMarker.pose.position.z = 0.;
        m_puckMarker.pose.orientation.w = 1.;
        m_puckMarker.pose.orientation.x = 0.;
        m_puckMarker.pose.orientation.y = 0.;
        m_puckMarker.pose.orientation.z = 0.;

        m_malletMarker.header.frame_id = "world";
        m_malletMarker.type = visualization_msgs::Marker::CYLINDER;
        m_malletMarker.action = visualization_msgs::Marker::MODIFY;
        m_malletMarker.scale.x = 0.0963;
        m_malletMarker.scale.y = 0.0963;
        m_malletMarker.scale.z = 0.065;
        m_malletMarker.color.r = 1.0;
        m_malletMarker.color.g = 0.0;
        m_malletMarker.color.b = 0.0;
        m_malletMarker.color.a = 1.0;
        m_malletMarker.pose.position.x = 0.;
        m_malletMarker.pose.position.y = 0.;
        m_malletMarker.pose.position.z = 0.;
        m_malletMarker.pose.orientation.w = 1.;
        m_malletMarker.pose.orientation.x = 0.;
        m_malletMarker.pose.orientation.y = 0.;
        m_malletMarker.pose.orientation.z = 0.;
    }

    ~VisualizationInterface()= default;

    bool lookupTF(){
        try{
            m_listener.lookupTransform("/world", "/Table", ros::Time(0), m_tfTable);
        }
        catch (tf::TransformException& ex) {
            return false;
        }

        try{
            m_listener.lookupTransform("/world", "/Puck", ros::Time(0), m_tfPuck);
        }
        catch (tf::TransformException& ex) {
            return false;
        }

        try{
            m_listener.lookupTransform("/world", "/Mallet", ros::Time(0), m_tfMallet);
            return true;
        }
        catch (tf::TransformException& ex) {
            return false;
        }
    }

    void castTFToMarker(const tf::StampedTransform& tf, visualization_msgs::Marker& marker){
        marker.header.stamp = tf.stamp_;
        marker.pose.position.x = tf.getOrigin().x();
        marker.pose.position.y = tf.getOrigin().y();
        marker.pose.position.z = tf.getOrigin().z();
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = 0.;
        marker.pose.orientation.w = 1.;
    }

    void visualize(){
        if(lookupTF()) {
            castTFToMarker(m_tfTable, m_tableMarker);
            castTFToMarker(m_tfPuck, m_puckMarker);
            castTFToMarker(m_tfMallet, m_malletMarker);
            m_tablePub.publish(m_tableMarker);
            m_puckPub.publish(m_puckMarker);
            m_malletPub.publish(m_malletMarker);
            m_predictionPub.publish(m_predictionMarker);
        }
    }

    void setPredictionMarker(const mrpt::math::CVectorDynamic<double>& prediction,
                             const mrpt::math::CMatrixDynamic<double>& variance){
        m_predictionMarker.header.frame_id = "world";
        m_predictionMarker.type = visualization_msgs::Marker::CYLINDER;
        m_predictionMarker.action = visualization_msgs::Marker::MODIFY;
        m_predictionMarker.scale.x = variance[0];
        m_predictionMarker.scale.y = variance[5];
        m_predictionMarker.scale.z = 0.01;
        m_predictionMarker.color.r = 0.0;
        m_predictionMarker.color.g = 0.0;
        m_predictionMarker.color.b = 1.0;
        m_predictionMarker.color.a = 0.2;
        m_predictionMarker.pose.position.x = prediction[0];
        m_predictionMarker.pose.position.y = prediction[1];
        m_predictionMarker.pose.position.z = m_tfPuck.getOrigin().z();
        m_predictionMarker.pose.orientation.w = 1.;
        m_predictionMarker.pose.orientation.x = 0.;
        m_predictionMarker.pose.orientation.y = 0.;
        m_predictionMarker.pose.orientation.z = 0.;
    }
};

#endif //PUCK_TRACKER_VISUALIZATION_INTERFACE_H
