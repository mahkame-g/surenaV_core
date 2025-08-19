#pragma once

#ifndef HAND_MANAGER_H
#define HAND_MANAGER_H

#include "S5_hand.h"
#include "MinimumJerkInterpolation.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <vector>
#include <string>
#include <iostream>
#include <fstream> 
#include <ros/ros.h>
#include <ros/package.h>
#include "json.hpp"

// ROS Message and Service Includes
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "hand_planner/DetectionInfoArray.h"
#include "hand_planner/move_hand_single.h"
#include "hand_planner/move_hand_both.h"
#include "hand_planner/gripOnline.h"
#include "hand_planner/home_service.h"
#include "hand_planner/SetTargetClass.h"

using namespace std;
using namespace Eigen;
using json = nlohmann::json;

class HandManager {
public:
    // --- CONSTRUCTOR ---
    HandManager(ros::NodeHandle *n);

private:
    // --- ROS COMMUNICATION HANDLES ---
    ros::Publisher trajectory_data_pub;
    ros::Subscriber camera_data_sub;
    ros::Subscriber joint_qc_sub;
    ros::ServiceServer move_hand_single_service;
    ros::ServiceServer move_hand_both_service;
    ros::ServiceServer grip_online_service;
    ros::ServiceServer home_service;
    ros::ServiceServer set_target_class_service;
    ros::Publisher gazeboJointStatePub_;
    std_msgs::Float64MultiArray joint_angles_gazebo_;

    // --- CORE OBJECTS ---
    S5_hand hand_func_R;
    S5_hand hand_func_L;
    MinimumJerkInterpolation coef_generator;

    // --- STATE & TRAJECTORY VARIABLES ---
    VectorXd q_ra;
    VectorXd q_la;
    VectorXd q_init_r;
    VectorXd q_init_l;
    MatrixXd qref_r;
    MatrixXd qref_l;
    VectorXd next_ini_ee_posR;
    VectorXd next_ini_ee_posL;
    double sum_r;
    double sum_l;
    int QcArr[29];

    // --- PARAMETERS ---
    double T;
    int rate;
    bool simulation;
    int encoderResolution[2];
    int harmonicRatio[4];
    vector<int> pitch_range, roll_range, yaw_range;
    vector<int> pitch_command_range, roll_command_range, yaw_command_range;
    vector<int> wrist_command_range, wrist_yaw_range, wrist_right_range, wrist_left_range;

    // --- VISION & ONLINE GRASPING VARIABLES ---
    double X, Y, Z;
    double tempX, tempY, tempZ;
    double h_pitch, h_roll, h_yaw;
    double Kp, Ky;
    double t_grip;
    int target_class_id_ = 41; // Default: "cup"
    std::mutex target_mutex_;

    // --- ROS CALLBACKS (Declarations) ---
    void object_detect_callback(const hand_planner::DetectionInfoArray &msg);
    void joint_qc_callback(const std_msgs::Int32MultiArray::ConstPtr &qcArray);

    // --- REFACTORED CORE LOGIC (Declarations) ---
    MatrixXd scenario_target(HandType type, string scenario, int i, VectorXd ee_pos, string ee_ini_pos);
    VectorXd reach_target(S5_hand& hand_model, VectorXd& q_arm, MatrixXd& qref_arm, double& sum_arm, VectorXd& q_init_arm, MatrixXd targets, string scenario, int M);

    // --- ROS SERVICE HANDLERS (Declarations) ---
    bool single_hand(hand_planner::move_hand_single::Request &req, hand_planner::move_hand_single::Response &res);
    bool both_hands(hand_planner::move_hand_both::Request &req, hand_planner::move_hand_both::Response &res);
    bool home(hand_planner::home_service::Request &req, hand_planner::home_service::Response &res);
    bool grip_online(hand_planner::gripOnline::Request &req, hand_planner::gripOnline::Response &res);
    bool setTargetClassService(hand_planner::SetTargetClass::Request &req, hand_planner::SetTargetClass::Response &res);
};

#endif // HAND_MANAGER_H