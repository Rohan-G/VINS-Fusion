/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <unistd.h>

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car, pub_imu, pub_gps;
nav_msgs::Path *global_path;
// double last_vio_t = -1;
// double last_imu_time = -1;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::mutex m_buf;
std::mutex feat_lock;
int num_features=100;
bool updated = false;
nav_msgs::Path imu_path_msg;
nav_msgs::Path gps_path_msg;

std::mutex lastUpd;
bool isRead = false;
double lastTime;
double last_vins_t = -1;

std::mutex updIMU;
double lastIMU = -1;
nav_msgs::OdometryConstPtr last_imu_msg = NULL;

Eigen::Matrix4d VINS_TRANSFORM = Eigen::Matrix4d::Identity();

void getCWD(){
    char buffer[PATH_MAX]; // To store the path
    if (getcwd(buffer, sizeof(buffer)) != NULL) {
        std::cout << "Current working directory: " << buffer << std::endl;
    } else {
        std::cerr << "Error getting current working directory." << std::endl;
    }
}

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    getCWD();

    car_mesh.mesh_resource = "/root/catkin_ws/src/VINS-Fusion/global_fusion/models/car.dae";

            // cout << "Size: " << feature.second.size() << "\n";
            // cout << "\n\n\n\n\n";
    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
            // cout << "Size: " << feature.second.size() << "\n";
            // cout << "\n\n\n\n\n";
            // cout << "Size: " << feature.second.size() << "\n";
            // cout << "\n\n\n\n\n";;
    pub_car.publish(markerArray_msg);
}

void feature_callback(const std_msgs::Int32ConstPtr &feat_msg)
{
    feat_lock.lock();
    updated = true;
    num_features = feat_msg->data;
    feat_lock.unlock();
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    // printf("gps_callback! \n");
    m_buf.lock();
    gpsQueue.push(GPS_msg);
    double xyz[3];
	globalEstimator.GPS2XYZ(GPS_msg->latitude, GPS_msg->longitude, GPS_msg->altitude, xyz);
    // cout << xyz[0] << " " << xyz[1] << " " << xyz[2] << endl;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = GPS_msg->header;
    pose_stamped.pose.position.x = xyz[0];
    pose_stamped.pose.position.y = xyz[1];
    pose_stamped.pose.position.z = xyz[2];
    gps_path_msg.header = pose_stamped.header;
    gps_path_msg.header.frame_id = "world";
    gps_path_msg.poses.push_back(pose_stamped);
    pub_gps.publish(gps_path_msg);
    m_buf.unlock();
    // cout << "Leaving GPS func\n";
}

void updMap(const nav_msgs::OdometryConstPtr &pose_msg, bool isIMU)
{

    // cout << isIMU << endl;
    
    double t = pose_msg->header.stamp.toSec();
    nav_msgs::OdometryConstPtr vins_msg = NULL;
    nav_msgs::OdometryConstPtr msg_to_use = NULL;

    // check for VINS reset
    bool vins_reset = false;
    lastUpd.lock();
    if(last_vins_t == -1){
        last_vins_t = t;
        msg_to_use = pose_msg;
    }
    else if(last_vins_t <= t - 1){
        updIMU.lock();
        last_vins_t = t;
        if(lastIMU > last_vins_t){
            vins_msg = pose_msg;
            msg_to_use = last_imu_msg;
            vins_reset = true;
            isIMU = true;
            updIMU.unlock();
        }
        else{
            cout << "System Failure!" << endl;
            updIMU.unlock();
            return;
        }
    }
    else{
        msg_to_use = pose_msg;
    }
    lastUpd.unlock();

    // cout << "Current Time: " << t << endl;
    printf("Current time : %lf, isIMU: %d\n", t, (int)isIMU);
    // last_imu_t = t;
    Eigen::Vector3d vio_t(msg_to_use->pose.pose.position.x, msg_to_use->pose.pose.position.y, msg_to_use->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = msg_to_use->pose.pose.orientation.w;
    vio_q.x() = msg_to_use->pose.pose.orientation.x;
    vio_q.y() = msg_to_use->pose.pose.orientation.y;
    vio_q.z() = msg_to_use->pose.pose.orientation.z;

    if(!isIMU){
        last_vins_t = t;
        Eigen::Matrix4d T;
        T.block<3,3>(0,0) = vio_q.toRotationMatrix();
        T.block<3,1>(0,3) = vio_t;
        T = VINS_TRANSFORM * T;
        vio_t = T.block<3,1>(0,3);
        vio_q = Eigen::Quaterniond(T.block<3,3>(0,0));
    }

    lastUpd.lock();
    lastTime = t;
    lastUpd.unlock();
    globalEstimator.inputOdom(t, vio_t, vio_q);

    m_buf.lock();
    bool gpsSeen = false;
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            //int numSats = GPS_msg->status.service;
            double pos_accuracy = GPS_msg->position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            // updMapprintf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
            globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            gpsSeen = true;
            break;
        }
        else if(gps_t < t - 0.01)
            gpsQueue.pop();
        else if(gps_t > t + 0.01)
            break;
    }
    if(!gpsSeen && !isIMU)
        globalEstimator.noGPS();

    else if(!gpsSeen){
        m_buf.unlock();
        cout << "Failure!" << endl;
        return;
    }
    
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    if(vins_reset){
        Eigen::Matrix3d R = Eigen::Quaterniond(vins_msg->pose.pose.orientation.w, vins_msg->pose.pose.orientation.x, vins_msg->pose.pose.orientation.y, vins_msg->pose.pose.orientation.z).toRotationMatrix();
        Eigen::Vector3d T = Eigen::Vector3d(vins_msg->pose.pose.position.x, vins_msg->pose.pose.position.y, vins_msg->pose.pose.position.z);
        Eigen::Matrix4d T1;
        Eigen::Matrix4d T2;
        T1.block<3,3>(0,0) = R;
        T1.block<3,1>(0,3) = T;

        T2.block<3,3>(0,0) = Eigen::Quaterniond(global_q.w(), global_q.x(), global_q.y(), global_q.z()).toRotationMatrix();
        T2.block<3,1>(0,3) = global_t;

        VINS_TRANSFORM = T2 * T1.inverse();
    }

    nav_msgs::Odometry odometry;
    odometry.header = msg_to_use->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    // publish_car_model(t, global_t, global_q);


    // write result to file
    std::ofstream foutC("/home/tony-ws1/output/vio_global.csv", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << msg_to_use->header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << global_t.x() << ","
            << global_t.y() << ","
            << global_t.z() << ","
            << global_q.w() << ","
            << global_q.x() << ","
            << global_q.y() << ","
            << global_q.z() << endl;
    foutC.close();
}

// IMU propagate's callback function
void imu_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    // printf("imu callback!\n");
    double t = pose_msg->header.stamp.toSec();

    lastUpd.lock();
    // cout << lastTime << " " << t << endl;
    if(!isRead){
        isRead = true;
        lastTime = t;
        lastUpd.unlock();
    }
    else if(t>=(lastTime+0.5)){
        lastUpd.unlock();
        updMap(pose_msg, true);
    }
    else{
        lastUpd.unlock();
        updIMU.lock();
        if(t > lastIMU || last_imu_msg == NULL){
            last_imu_msg = pose_msg;
        }
        updIMU.unlock();
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position = pose_msg->pose.pose.position;
    pose_stamped.pose.orientation = pose_msg->pose.pose.orientation;
    imu_path_msg.header = pose_msg->header;
    imu_path_msg.header.frame_id = "world";
    imu_path_msg.poses.push_back(pose_stamped);
    pub_imu.publish(imu_path_msg);
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    // cout << "VIO callback\n";
    updMap(pose_msg, false);
    
}

int main(int argc, char **argv)
{

    printf("Starting Fusion\n");

    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    global_path = &globalEstimator.global_path;

    ros::Subscriber sub_GPS = n.subscribe("/mavros/global_position/raw/fix", 100, GPS_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    // ros::Subscriber sub_n_feat = n.subscribe("/vins_estimator/num_features", 100, feature_callback);
    ros::Subscriber sub_imu = n.subscribe("/vins_estimator/imu_propagate", 100, imu_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_imu = n.advertise<nav_msgs::Path>("imu",100);
    pub_gps = n.advertise<nav_msgs::Path>("gps",100);
    // pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    ros::spin();
    return 0;
}
