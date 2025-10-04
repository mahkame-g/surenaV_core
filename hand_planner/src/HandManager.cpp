#include "HandManager.h"

// --- CONSTRUCTOR ---
HandManager::HandManager(ros::NodeHandle *n) : 
    hand_func_R(RIGHT), 
    hand_func_L(LEFT),
    // Initialize parameters
    T(0.005),
    rate(200),
    simulation(false),
    X(1.0), Y(0.0), Z(0.0),
    tempX(1.0), tempY(0.0), tempZ(0.0),
    h_pitch(0), h_roll(0), h_yaw(0),
    Kp(0.01), Ky(-0.01),
    t_grip(0),
    sum_r(0), sum_l(0)
{
    // Motor and sensor constants
    encoderResolution[0] = 4096 * 4;
    encoderResolution[1] = 2048 * 4;
    harmonicRatio[0] = 100;
    harmonicRatio[1] = 100;
    harmonicRatio[2] = 100;
    harmonicRatio[3] = 400;
    
    // Parameter vectors
    pitch_range = {-30, 30};
    roll_range = {-50, 50};
    yaw_range = {-90, 90};
    pitch_command_range = {180, 140};
    roll_command_range = {100, 200};
    yaw_command_range = {90, 210};
    wrist_command_range = {0, 180};
    wrist_yaw_range = {90, -90};
    wrist_right_range = {90, -90};
    wrist_left_range = {90, -90};

    q_rad_teleop.resize(14);
    q_rad_teleop.setZero();

    // initial position
    q_right_state_.resize(7);
    q_right_state_ << 10*M_PI/180.0, -10*M_PI/180.0, 0, -25*M_PI/180.0, 0, 0, 0;
    R_right_state_.setIdentity();
    right_state_init_ = true;
    
    // Initialize finger control
    finger_control_ = std::make_unique<FingerControl>(n);

    // ROS Communication Setup
    trajectory_data_pub = n->advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
    gazeboJointStatePub_ = n->advertise<std_msgs::Float64MultiArray>("/joint_angles_gazebo", 100);
    camera_data_sub = n->subscribe("/detection_info", 1, &HandManager::object_detect_callback, this);
    joint_qc_sub = n->subscribe("jointdata/qc", 100, &HandManager::joint_qc_callback, this);
    teleoperation_data_sub = n->subscribe("teleoperation/angles", 100, &HandManager::teleoperation_callback, this);
    micArray_data_sub = n->subscribe("micarray/angle", 100, &HandManager::micArray_callback, this);
    move_hand_single_service = n->advertiseService("move_hand_single_srv", &HandManager::single_hand, this);
    move_hand_both_service = n->advertiseService("move_hand_both_srv", &HandManager::both_hands, this);
    grip_online_service = n->advertiseService("grip_online_srv", &HandManager::grip_online, this);
    home_service = n->advertiseService("home_srv", &HandManager::home, this);
    set_target_class_service = n->advertiseService("set_target_class_srv", &HandManager::setTargetClassService, this);
    head_track_service = n->advertiseService("head_track_srv", &HandManager::head_track_handler, this);
    teleoperation_service = n->advertiseService("teleoperation_srv", &HandManager::teleoperation_handler, this);
    write_string_service_        = n->advertiseService("write_string_srv",        &HandManager::write_string_handler, this);
    move_hand_relative_service_  = n->advertiseService("move_hand_relative_srv",  &HandManager::move_hand_relative_handler, this);
    move_hand_keyboard_service_  = n->advertiseService("move_hand_keyboard_srv",  &HandManager::move_hand_keyboard_handler, this);
    move_hand_general_service_   = n->advertiseService("move_hand_general_srv",   &HandManager::move_hand_general_handler, this);
    
    // Finger control services
    finger_control_service_ = n->advertiseService("finger_control_srv", &HandManager::fingerControlService, this);
    finger_scenario_service_ = n->advertiseService("finger_scenario_srv", &HandManager::fingerScenarioService, this);
}

// --- Object Detection Callback Implementations ---
bool HandManager::setTargetClassService(hand_planner::SetTargetClass::Request &req, hand_planner::SetTargetClass::Response &res) {
        // Look up class ID from JSON
        string object_classes_path = ros::package::getPath("hand_planner") + "/config/object_classes.json";
        std::ifstream fr(object_classes_path);
        json object_classes = json::parse(fr);
        if (!object_classes.contains(req.class_name)) {
            ROS_ERROR("Class name '%s' not found in object_classes.json!", req.class_name.c_str());
            res.class_id = -1;
        } else {
            target_class_id_ = object_classes[req.class_name];    
            res.class_id = target_class_id_;
            X = 1.0; Y = 0; Z = 0; 
        }
        return true;
}

void HandManager::object_detect_callback(const hand_planner::DetectionInfoArray &msg) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < msg.detections.size(); ++i) {
        if (msg.detections[i].class_id == target_class_id_) {
            double dist = msg.detections[i].distance / 1000.0;
            double y_pixel = msg.detections[i].x + msg.detections[i].width / 2.0;
            double z_pixel = msg.detections[i].y + msg.detections[i].height / 2.0;
            double a = 0.5, b = 0.4, X0 = 0.5;
            int L = 640, W = 480;

            double Y0 = -(y_pixel - L / 2.0) / L * a;
            double Z0 = -(z_pixel - W / 2.0) / W * b;
            double L0 = sqrt(pow(X0, 2) + pow(Y0, 2) + pow(Z0, 2));

            X = X0 * dist / L0;
            Y = Y0 * dist / L0;
            Z = Z0 * dist / L0;
            tempX = X; tempY = Y; tempZ = Z;
            return;
        }
    }
    X = tempX; Y = tempY; Z = tempZ;
}

void HandManager::joint_qc_callback(const std_msgs::Int32MultiArray::ConstPtr &qcArray) {
    for (size_t i = 0; i < qcArray->data.size() && i < 29; ++i) {
        QcArr[i] = qcArray->data[i];
        // cout << QcArr[i] << " ";
    }
    // cout << endl;
}

void HandManager::teleoperation_callback(const std_msgs::Float64MultiArray &q_deg_teleop) {
    assert(q_deg_teleop.data.size() == 14);
    for (size_t i = 0; i < q_deg_teleop.data.size(); ++i) {
        q_rad_teleop(i) = q_deg_teleop.data[i] * M_PI / 180.0;  // deg → rad
    }
}

void HandManager::micArray_callback(const std_msgs::Float64 &msg) {
    int MAX_CAPACITY = 10;
    micArray_data_buffer.push_back(msg.data);
    if (micArray_data_buffer.size() > MAX_CAPACITY) {
        micArray_data_buffer.pop_front();
    }
    // Only check if we have at least 5 values
    if (micArray_data_buffer.size() >= 3) {
        auto start = micArray_data_buffer.end() - 3;    // Take last 3 elements
        bool all_equal = std::all_of(start + 1, micArray_data_buffer.end(), [first = *start](double v){ return fabs(v - first) < 1e-6; });
        if (all_equal) {
            micArray_theta = (msg.data - 90) * M_PI / 180;
        }
    }
}

MatrixXd HandManager::scenario_target(HandType type, string scenario, int i, VectorXd ee_pos, string ee_ini_pos) {
    MatrixXd result(6, 3);
    VectorXd r_middle, r_target, r_start;
    MatrixXd R_target;

    S5_hand& hand_func = (type == RIGHT) ? hand_func_R : hand_func_L;
    VectorXd& q_arm = (type == RIGHT) ? q_ra : q_la;
    VectorXd& q_init = (type == RIGHT) ? q_init_r : q_init_l;
    VectorXd& next_ee_pos = (type == RIGHT) ? next_ini_ee_posR : next_ini_ee_posL;

    if (scenario == "shakeHands") {
        r_middle = (type == RIGHT) ? Vector3d(0.35, -0.1, -0.2) : Vector3d(0.35, 0.1, -0.2);
        r_target = (type == RIGHT) ? Vector3d(0.3, -0.03, -0.3) : Vector3d(0.3, 0.03, -0.3);
        R_target = hand_func.rot(2, -65 * M_PI / 180, 3);
    } else if (scenario == "respect") {
        r_middle = (type == RIGHT) ? Vector3d(0.3, -0.1, -0.3) : Vector3d(0.3, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, 0.2, -0.3) : Vector3d(0.3, -0.1, -0.3);
        double rot_angle = (type == RIGHT) ? 60 * M_PI / 180 : -60 * M_PI / 180;
        R_target = hand_func.rot(2, -80 * M_PI / 180, 3) * hand_func.rot(1, rot_angle, 3);
    } else if (scenario == "byebye") {
        r_middle = (type == RIGHT) ? Vector3d(0.35, -0.2, -0.15) : Vector3d(0.35, 0.2, -0.15);
        r_target = (type == RIGHT) ? Vector3d(0.3, -0.1, 0.22) : Vector3d(0.3, 0.1, 0.22);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot( (type == RIGHT ? 1 : 2), -180 * M_PI / 180, 3);
    } else if (scenario == "punching") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.1) : Vector3d(0.15, 0.1, -0.1);
        r_target = (type == RIGHT) ? Vector3d(0.35, 0.2, 0.1) : Vector3d(0.35, -0.2, 0.1);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -40 * M_PI / 180, 3);
    } else if (scenario == "perfect") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.3) : Vector3d(0.15, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, -0.05, -0.25) : Vector3d(0.25, 0.05, -0.25);
        R_target = hand_func.rot(2, -90 * M_PI / 180, 3) * hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -45 * M_PI / 180, 3);
    } else if (scenario == "pointing") {
        r_middle = (type == RIGHT) ? Vector3d(0.25, -0.1, -0.1) : Vector3d(0.25, 0.1, -0.1);
        r_target = (type == RIGHT) ? Vector3d(0.45, 0.05, 0.0) : Vector3d(0.45, -0.05, 0.0);
        R_target = hand_func.rot(3, 90 * M_PI / 180, 3) * hand_func.rot(1, -65 * M_PI / 180, 3);
    } else if (scenario == "like") {
        r_middle = (type == RIGHT) ? Vector3d(0.15, -0.1, -0.3) : Vector3d(0.15, 0.1, -0.3);
        r_target = (type == RIGHT) ? Vector3d(0.25, -0.05, -0.25) : Vector3d(0.25, 0.05, -0.25);
        R_target = hand_func.rot(2, -90 * M_PI / 180, 3);
    } else if (scenario == "home") {
        r_middle = (type == RIGHT) ? Vector3d(0.3, -0.1, -0.25) : Vector3d(0.3, 0.1, -0.25);
        r_target = (type == RIGHT) ? Vector3d(0.15, -0.07, -0.43) : Vector3d(0.15, 0.07, -0.43);
        R_target = hand_func.rot(2, -20 * M_PI / 180, 3);
    }

    q_arm.resize(7);
    q_init.resize(7);
    if (i == 0) {
        if (ee_ini_pos == "init") {
            if (type == RIGHT) {
                q_arm << 10, -10, 0, -25, 0, 0, 0;
            } else { // LEFT
                q_arm << 10, 10, 0, -25, 0, 0, 0;
            }
            q_arm *= M_PI / 180.0;
            q_init = q_arm;
            hand_func.HO_FK_palm(q_arm);
            r_start = hand_func.r_palm;
        } else { 
            r_start = next_ee_pos; }
    } else { 
        r_start = ee_pos; }

    result << r_middle.transpose(), r_target.transpose(), R_target.row(0), R_target.row(1), R_target.row(2), r_start.transpose();
    return result;
}

VectorXd HandManager::reach_target(S5_hand& hand_model, VectorXd& q_arm, MatrixXd& qref_arm, double& sum_arm, VectorXd& q_init_arm, MatrixXd targets, string scenario, int M) {
    qref_arm.resize(7, M);
    Vector3d r_middle = targets.row(0), r_target = targets.row(1), r_start = targets.row(5);
    Matrix3d R_target = targets.block(2, 0, 3, 3);
    
    MatrixXd t_r(1, 3); 
    t_r << 0, 2, 4;
    double total_time = (scenario == "byebye" || scenario == "shakeHands") ? t_r(2) + 4 : t_r(2);
    int step_count = total_time / T;
    
    MatrixXd P_x(1, 3), P_y(1, 3), P_z(1, 3);
    P_x << r_start(0), r_middle(0), r_target(0);
    P_y << r_start(1), r_middle(1), r_target(1);
    P_z << r_start(2), r_middle(2), r_target(2);
    MatrixXd V_inf(1,3); V_inf << 0, INFINITY, 0; 
    MatrixXd A_inf(1,3); A_inf << 0, INFINITY, 0;

    // define minJerk elements to calculate end effector velocity
    MatrixXd X_coef = coef_generator.Coefficient(t_r, P_x, V_inf, A_inf);
    MatrixXd Y_coef = coef_generator.Coefficient(t_r, P_y, V_inf, A_inf);
    MatrixXd Z_coef = coef_generator.Coefficient(t_r, P_z, V_inf, A_inf);

    for (int count = 0; count < step_count; ++count) {
        double time_r = count * T;
        Vector3d V_curr;
        if (time_r < t_r(2)) {
            int interval = (time_r < t_r(1)) ? 0 : 1;
            double t_interval = (time_r < t_r(1)) ? t_r(0) : t_r(1);
            V_curr << coef_generator.GetAccVelPos(X_coef.row(interval), time_r, t_interval, 5)(0,1),
                      coef_generator.GetAccVelPos(Y_coef.row(interval), time_r, t_interval, 5)(0,1),
                      coef_generator.GetAccVelPos(Z_coef.row(interval), time_r, t_interval, 5)(0,1);
            hand_model.update_hand(q_arm, V_curr, r_target, R_target);
        } else {
            if (scenario == "byebye") { 
                q_arm(2) += (hand_model.hand_type == RIGHT ? -0.5 : 0.5) * M_PI/180 * cos((time_r - t_r(2)) * (2 * M_PI)); }
            else if (scenario == "shakeHands") { 
                q_arm(3) -= 0.125 * M_PI/180 * cos((time_r - t_r(2)) * M_PI); }
            hand_model.update_hand(q_arm, Vector3d::Zero(), r_target, R_target);
        }
        hand_model.doQP(q_arm);
        q_arm = hand_model.q_next;
        int global_index = count + sum_arm / T;
        if (global_index < qref_arm.cols()) { 
            qref_arm.col(global_index) = q_arm - q_init_arm; }
    }
    sum_arm += total_time;
    hand_model.HO_FK_palm(q_arm);
    return hand_model.r_palm;
}

void HandManager::publishMotorData(const VectorXd& q_rad_right, const VectorXd& q_rad_left, const Vector3d& head_angles) {
     /* The function automatically:
      * - Converts radians to encoder counts for real robot
      * - Handles wrist calculations
      * - Publishes to appropriate topics (/jointdata/qc or /joint_angles_gazebo)
      * - Supports both simulation and real robot modes
      * - ALWAYS publishes data for all 29 joints (use zero vectors for joints you don't want to control)
    */
    vector<double> q_motor(29, 0.0);
    vector<double> q_gazebo(29, 0.0);
    
    if (!simulation) {
        // Right hand motors (indices 12-15)
        q_motor[12] = int(q_rad_right(0) * encoderResolution[0] * harmonicRatio[0] / (2 * M_PI));
        q_motor[13] = -int(q_rad_right(1) * encoderResolution[0] * harmonicRatio[1] / (2 * M_PI));
        q_motor[14] = int(q_rad_right(2) * encoderResolution[1] * harmonicRatio[2] / (2 * M_PI));
        q_motor[15] = -int(q_rad_right(3) * encoderResolution[1] * harmonicRatio[3] / (2 * M_PI));
        
        // Left hand motors (indices 16-19)
        q_motor[16] = -int(q_rad_left(0) * encoderResolution[0] * harmonicRatio[0] / (2 * M_PI));
        q_motor[17] = -int(q_rad_left(1) * encoderResolution[0] * harmonicRatio[1] / (2 * M_PI));
        q_motor[18] = int(q_rad_left(2) * encoderResolution[1] * harmonicRatio[2] / (2 * M_PI));
        q_motor[19] = int(q_rad_left(3) * encoderResolution[1] * harmonicRatio[3] / (2 * M_PI));
        
        // Head motors (indices 20-22) - roll, pitch, yaw
        q_motor[20] = int(roll_command_range[0] + (roll_command_range[1] - roll_command_range[0]) * 
                         ((-(head_angles(0)*180/M_PI) - roll_range[0]) / (roll_range[1] - roll_range[0])));
        q_motor[21] = int(pitch_command_range[0] + (pitch_command_range[1] - pitch_command_range[0]) * 
                         ((-(head_angles(1)*180/M_PI) - pitch_range[0]) / (pitch_range[1] - pitch_range[0])));
        q_motor[22] = int(yaw_command_range[0] + (yaw_command_range[1] - yaw_command_range[0]) * 
                         ((-(head_angles(2)*180/M_PI) - yaw_range[0]) / (yaw_range[1] - yaw_range[0])));
        
        // Wrist calculations for right hand (indices 23-25)
        q_motor[23] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((q_rad_right(4) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
        q_motor[24] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((hand_func_R.wrist_right_calc(q_rad_right(5), q_rad_right(6))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
        q_motor[25] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((hand_func_R.wrist_left_calc(q_rad_right(5), q_rad_right(6))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));
    
        // Wrist calculations for left hand (indices 26-28)
        q_motor[26] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((q_rad_left(4) * 180 / M_PI) - wrist_yaw_range[0]) / (wrist_yaw_range[1] - wrist_yaw_range[0])));
        q_motor[27] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((hand_func_L.wrist_right_calc(q_rad_left(5), q_rad_left(6))) - (wrist_right_range[0])) / (wrist_right_range[1] - (wrist_right_range[0]))));
        q_motor[28] = int(wrist_command_range[0] + (wrist_command_range[1] - wrist_command_range[0]) * 
                            (((hand_func_L.wrist_left_calc(q_rad_left(5), q_rad_left(6))) - wrist_left_range[0]) / (wrist_left_range[1] - wrist_left_range[0])));
        
        // Publish motor data
        std_msgs::Int32MultiArray trajectory_data;
        for(int i = 0; i < 29; i++) { 
            trajectory_data.data.push_back(q_motor[i]); 
        }
        trajectory_data_pub.publish(trajectory_data);
        // cout << q_motor[12] << ',' << q_motor[13] << ',' << q_motor[14] << ',' << q_motor[15] << ','<< q_motor[23] << ','<< q_motor[24] << ','<< q_motor[25] <<endl;
    } else { // simulation
        // Right hand joints
        q_gazebo[12] = q_rad_right(0);  
        q_gazebo[13] = q_rad_right(1);   
        q_gazebo[14] = q_rad_right(2);  
        q_gazebo[15] = q_rad_right(3);
        
        // Left hand joints
        q_gazebo[16] = q_rad_left(0);  
        q_gazebo[17] = q_rad_left(1);   
        q_gazebo[18] = q_rad_left(2);  
        q_gazebo[19] = q_rad_left(3);
        
        // Head joints
        q_gazebo[20] = -head_angles(0); // roll
        q_gazebo[21] = -head_angles(1); // pitch
        q_gazebo[22] = -head_angles(2); // yaw
        
        // Wrist joints for right hand
        q_gazebo[23] = q_rad_right(4);   
        q_gazebo[24] = q_rad_right(5);  
        q_gazebo[25] = q_rad_right(6);
        
        // Wrist joints for left hand
        q_gazebo[26] = q_rad_left(4);   
        q_gazebo[27] = q_rad_left(5);  
        q_gazebo[28] = q_rad_left(6); 
        
        // Publish gazebo data
        joint_angles_gazebo_.data.clear();
        for (int i = 0; i < 29; i++) {
            joint_angles_gazebo_.data.push_back(q_gazebo[i]);
        }
        gazeboJointStatePub_.publish(joint_angles_gazebo_);
    }
}

// --- Service Handler Implementations ---
bool HandManager::single_hand(hand_planner::move_hand_single::Request &req, hand_planner::move_hand_single::Response &res) {
    ros::Rate rate_(rate);
    int M = req.t_total / T;
    VectorXd ee_pos = Vector3d::Zero();
    HandType type = (req.mode == "righthand") ? RIGHT : LEFT;
    if (type == RIGHT) {
        for (int i = 0; i < req.scen_count; i++) {
            MatrixXd result = scenario_target(RIGHT, req.scenario[i], i, ee_pos, req.ee_ini_pos);
            ee_pos = reach_target(hand_func_R, q_ra, qref_r, sum_r, q_init_r, result, req.scenario[i], M);
        }
    } else {
        for (int i = 0; i < req.scen_count; i++) {
            MatrixXd result = scenario_target(LEFT, req.scenario[i], i, ee_pos, req.ee_ini_pos);
            ee_pos = reach_target(hand_func_L, q_la, qref_l, sum_l, q_init_l, result, req.scenario[i], M);
        }
    }

    for (int id = 0; id < M; ++id) {
        if (type == RIGHT) {
            VectorXd q_rad = qref_r.col(id);
            VectorXd q_rad_left = VectorXd::Zero(7);
            Vector3d head_angles(0, 0, 0);
            publishMotorData(q_rad, q_rad_left, head_angles);
        } else {
            VectorXd q_rad_right = VectorXd::Zero(7);
            VectorXd q_rad = qref_l.col(id);
            Vector3d head_angles(0, 0, 0);
            publishMotorData(q_rad_right, q_rad, head_angles);
        }
        ros::spinOnce();
        rate_.sleep();
    }

    sum_r = 0; sum_l = 0;
    if (type == RIGHT) { 
        next_ini_ee_posR = ee_pos; } 
    else { 
        next_ini_ee_posL = ee_pos; }
    res.ee_fnl_pos = req.scenario[req.scen_count - 1];
    return true;
}

bool HandManager::both_hands(hand_planner::move_hand_both::Request &req, hand_planner::move_hand_both::Response &res) {
    ros::Rate rate_(rate);
    int M = req.t_total / T;
    VectorXd ee_pos_r = Vector3d::Zero();
    VectorXd ee_pos_l = Vector3d::Zero();

    // --- Trajectory Generation Phase ---
    // Generate trajectory for the right hand
    for (int i = 0; i < req.scenR_count; i++) {
        MatrixXd result_r = scenario_target(RIGHT, req.scenarioR[i], i, ee_pos_r, req.ee_ini_posR);
        ee_pos_r = reach_target(hand_func_R, q_ra, qref_r, sum_r, q_init_r, result_r, req.scenarioR[i], M);
    }
    // Generate trajectory for the left hand
    for (int i = 0; i < req.scenL_count; i++) {
        MatrixXd result_l = scenario_target(LEFT, req.scenarioL[i], i, ee_pos_l, req.ee_ini_posL);
        ee_pos_l = reach_target(hand_func_L, q_la, qref_l, sum_l, q_init_l, result_l, req.scenarioL[i], M);
    }

    // --- Execution Phase ---
    for (int id = 0; id < M; ++id) {
        VectorXd q_rad_r = qref_r.col(id);
        VectorXd q_rad_l = qref_l.col(id);
        Vector3d head_angles(0, 0, 0);
        publishMotorData(q_rad_r, q_rad_l, head_angles);
        ros::spinOnce();
        rate_.sleep();
    }

    // --- Cleanup and Service Response ---
    sum_r = 0;
    sum_l = 0;
    next_ini_ee_posR = ee_pos_r;
    next_ini_ee_posL = ee_pos_l;

    res.ee_fnl_posR = req.scenarioR.empty() ? "" : req.scenarioR[req.scenR_count - 1];
    res.ee_fnl_posL = req.scenarioL.empty() ? "" : req.scenarioL[req.scenL_count - 1];
    return true;
}

bool HandManager::home(hand_planner::home_service::Request &req, hand_planner::home_service::Response &res) {
    ROS_WARN("Home service is not fully implemented yet.");
    return true;
}

bool HandManager::grip_online(hand_planner::gripOnline::Request &req, hand_planner::gripOnline::Response &res) {
    ros::Rate rate_(rate);
    VectorXd current_q_ra(7);
    current_q_ra << q_right_state_;
    VectorXd initial_q_ra = current_q_ra;

    Matrix3d R_target_r = Matrix3d::Identity();

    t_grip = 0;
    while (t_grip <= (120)) {
        Vector3d target2camera(X, Y, Z);
        MatrixXd T_CAM2SH = hand_func_R.ObjToNeck(-h_pitch, h_roll, -h_yaw);
        Vector3d target2shoulder = T_CAM2SH.block(0, 3, 3, 1) + T_CAM2SH.block(0, 0, 3, 3) * target2camera;
        
        // Head Pitch
        if (abs(target2camera(2)) > 0.02) {
            h_pitch += Kp * atan2(target2camera(2), sqrt(pow(target2camera(1),2) + pow(target2camera(0),2)));
            h_pitch = max(-28.0*M_PI/180, min(28.0*M_PI/180, h_pitch));
        }
        // Head Yaw
        if (abs(target2camera(1)) > 0.02) {
            h_yaw += Ky * atan2(target2camera(1), target2camera(0));
            h_yaw = max(-60.0*M_PI/180, min(60.0*M_PI/180, h_yaw));
        }

        if (t_grip >= 15 && Y != 0 && Z != 0) {
        R_target_r = hand_func_R.rot(2, -65 * M_PI / 180, 3);
        hand_func_R.update_hand(current_q_ra, Vector3d::Zero(), target2shoulder, R_target_r);
        Vector3d V_r = 0.7 * (target2shoulder - hand_func_R.r_palm);
        
        hand_func_R.update_hand(current_q_ra, V_r, target2shoulder, R_target_r);
        hand_func_R.doQP(current_q_ra);  // Solve the inverse kinematics
        current_q_ra = hand_func_R.q_next;
        }

        VectorXd q_delta = current_q_ra - initial_q_ra;
        VectorXd q_rad_left = VectorXd::Zero(7);
        Vector3d head_angles(h_roll, h_pitch, h_yaw);
        publishMotorData(q_delta, q_rad_left, head_angles);
        ros::spinOnce();
        rate_.sleep();
        t_grip += T;
    }


    res.finish = "end";
    q_right_state_= current_q_ra;
    return true;
}

bool HandManager::head_track_handler(hand_planner::head_track::Request &req, hand_planner::head_track::Response &res) {
    ROS_INFO("Starting head tracking for %.2f seconds.", req.duration_seconds);
    ros::Rate rate_(rate);
    ros::Time start_time = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < req.duration_seconds) {
        Vector3d target2camera(X, Y, Z);
        
        if (Y != 0 && Z != 0){ // target not in robot's sight; so the head cannot track object yet.
            // Head Pitch
            if (abs(target2camera(2)) > 0.02) {
                h_pitch += Kp * atan2(target2camera(2), sqrt(pow(target2camera(1), 2) + pow(target2camera(0), 2)));
                h_pitch = max(-28.0 * M_PI / 180, min(28.0 * M_PI / 180, h_pitch));
            }
            // Head Yaw
            if (abs(target2camera(1)) > 0.02) {
                h_yaw += Ky * atan2(target2camera(1), target2camera(0));
                h_yaw = max(-60.0 * M_PI / 180, min(60.0 * M_PI / 180, h_yaw));
            }
        } else { // follow direction of arrival (voice) instead.
            h_yaw += Ky * micArray_theta;
            h_yaw = max(-60.0 * M_PI / 180, min(60.0 * M_PI / 180, h_yaw));
        }

        VectorXd q_rad_right = VectorXd::Zero(7);
        VectorXd q_rad_left = VectorXd::Zero(7);
        Vector3d head_angles(h_roll, h_pitch, h_yaw);
        publishMotorData(q_rad_right, q_rad_left, head_angles);
        ros::spinOnce();
        rate_.sleep();
    }
    ROS_INFO("Head tracking finished.");
    res.success = true;
    return true;
}

bool HandManager::teleoperation_handler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ros::Rate rate_(rate);
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 120) {
        VectorXd q_rad_right = q_rad_teleop.segment(0, 7);
        VectorXd q_rad_left = q_rad_teleop.segment(7, 7);
        Vector3d head_angles(0, 0, 0);
        publishMotorData(q_rad_right, q_rad_left, head_angles);
        ros::spinOnce();
        rate_.sleep();
    }
    return true;
}


bool HandManager::write_string_handler(hand_planner::WriteString::Request &req, hand_planner::WriteString::Response &res) {
    VectorXd q(7);
    q << q_right_state_;

    Vector3d r_target(0.45, 0.02, 0.03);
    Matrix3d R_target = hand_func_R.rot(2, -140.0*M_PI/180.0, 3)
                      * hand_func_R.rot(1,  -25.0*M_PI/180.0, 3)
                      * hand_func_R.rot(3,   30.0*M_PI/180.0, 3);

    auto pub = [&](const VectorXd& qr){
        VectorXd ql = VectorXd::Zero(7);
        publishMotorData(qr, ql, Vector3d(0,0,0));
    };

    if (!approachWhiteboard(hand_func_R, coef_generator, q, r_target, R_target, T, pub)) { res.success=false; return true; }
    writeStringCore(hand_func_R, q, req.data, r_target, R_target, T, pub);
    res.success = true;
    q_right_state_ = q;
    return true;
}

bool HandManager::move_hand_relative_handler(hand_planner::PickAndMove::Request &req, hand_planner::PickAndMove::Response &res) {
    VectorXd q(7);
    q << 10*M_PI/180.0, -10*M_PI/180.0, 0, -25*M_PI/180.0, 0, 0, 0;

    Matrix3d R_pick = hand_func_R.rot(2, -100.0*M_PI/180.0, 3);
    Vector3d mid(0.15, -0.15, -0.30);
    Vector3d goal(0.35, -0.10, -0.20);

    auto pub = [&](const VectorXd& qr){
        VectorXd ql = VectorXd::Zero(7);
        publishMotorData(qr, ql, Vector3d(0,0,0));
    };

    if (!approachViaOneMid(hand_func_R, coef_generator, q, mid, goal, R_pick, 3.0, 3.0, T, pub)) {
        res.ok = false; res.message = "approach failed"; return true;
    }

    if (req.axes.size() != req.deltas.size() || req.axes.size() != req.durations.size()) {
        res.ok=false; res.message="size mismatch"; return true;
    }

    double MAX_DX = 0.08, MAX_DYZ = 0.30, MIN_DUR = 0.2;
    for (size_t i=0;i<req.axes.size();++i){
        char ax=0; for(char c: req.axes[i]){ if (std::isalpha((unsigned char)c)){ char u=std::toupper((unsigned char)c); if(u=='X'||u=='Y'||u=='Z'){ ax=u; break; } } }
        if (!ax) continue;
        double d=req.deltas[i], dur=std::max(req.durations[i], MIN_DUR);
        Vector3d dxyz(0,0,0);
        if (ax=='X'){ d = std::max(std::min(d, MAX_DX), -MAX_DX); dxyz(0)=d; }
        else if (ax=='Y'){ d = std::max(std::min(d, MAX_DYZ), -MAX_DYZ); dxyz(1)=d; }
        else { d = std::max(std::min(d, MAX_DYZ), -MAX_DYZ); dxyz(2)=d; }
        if (!moveRelative(hand_func_R, coef_generator, q, dxyz, goal, R_pick, dur, T, pub)) { res.ok=false; res.message="move failed"; return true; }
    }

    res.ok=true; res.message="ok";
    q_right_state_ = q;
    return true;
}

bool HandManager::move_hand_keyboard_handler(hand_planner::KeyboardJog::Request &req,
                                             hand_planner::KeyboardJog::Response &res)
{
    VectorXd q(7);
    q << q_right_state_;
    S5_hand& H = hand_func_R;
    H.HO_FK_palm(q);

    {
        Eigen::Vector3d mid(0.15, -0.15, -0.30);
        Eigen::Vector3d goal(0.35, -0.10, -0.20);
        Matrix3d Rg = H.rot(2, -100.0*M_PI/180.0, 3);
        double T1 = 3.0, T2 = 3.0;
        MinimumJerkInterpolation mj;
        MatrixXd t(1,3); t<<0.0, T1, T1+T2;

        H.HO_FK_palm(q);
        Vector3d r0 = H.r_palm;
        MatrixXd Px(1,3),Py(1,3),Pz(1,3); Px<<r0(0),mid(0),goal(0); Py<<r0(1),mid(1),goal(1); Pz<<r0(2),mid(2),goal(2);
        const double INF = std::numeric_limits<double>::infinity();
        MatrixXd Vx(1,3),Vy(1,3),Vz(1,3),Ax(1,3),Ay(1,3),Az(1,3);
        Vx<<0,INF,0; Vy<<0,INF,0; Vz<<0,INF,0; Ax<<0,INF,0; Ay<<0,INF,0; Az<<0,INF,0;
        MatrixXd ord(1,2); ord.fill(5);
        MatrixXd conx(3,3),cony(3,3),conz(3,3); conx<<Px,Vx,Ax; cony<<Py,Vy,Ay; conz<<Pz,Vz,Az;

        MatrixXd Xc = mj.Coefficient1(t, ord, conx, 0.1).transpose();
        MatrixXd Yc = mj.Coefficient1(t, ord, cony, 0.1).transpose();
        MatrixXd Zc = mj.Coefficient1(t, ord, conz, 0.1).transpose();

        ros::Rate rate_(rate);
        for (double tt=0; tt<(T1+T2); tt+=T) {
            int seg = (tt < T1) ? 0 : 1;
            Vector3d V(
                mj.GetAccVelPos(Xc.row(seg), tt, 0, 5)(0,1),
                mj.GetAccVelPos(Yc.row(seg), tt, 0, 5)(0,1),
                mj.GetAccVelPos(Zc.row(seg), tt, 0, 5)(0,1)
            );
            H.update_hand(q, V, goal, Rg);
            H.doQP(q);
            q = H.q_next;

            VectorXd qL = VectorXd::Zero(7);
            Vector3d head(0,0,0);
            publishMotorData(q, qL, head);

            ros::spinOnce();
            rate_.sleep();
        }
    }

    termios oldt; bool tty_ok=false;
    {
        if (tcgetattr(STDIN_FILENO, &oldt) == 0) {
            termios newt = oldt; newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
            tty_ok = true;
        }
    }

    auto restoreTTY = [&](){
        if (tty_ok) tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    };

    ros::Rate rate_(rate);
    ros::WallTime last_input = ros::WallTime::now();
    const double STEP_T = 1.0; 
    const double MAX_DX = 0.08;      
    const double MAX_DYZ = 0.30;   

    auto doRelative = [&](double dx, double dy, double dz, double dur){
        H.HO_FK_palm(q);
        Vector3d r0 = H.r_palm;
        MinimumJerkInterpolation mj;

        MatrixXd t(1,2); t<<0.0, dur;
        MatrixXd Px(1,2),Py(1,2),Pz(1,2);
        Px<<r0(0), r0(0)+dx;
        Py<<r0(1), r0(1)+dy;
        Pz<<r0(2), r0(2)+dz;

        MatrixXd Vx(1,2),Vy(1,2),Vz(1,2),Ax(1,2),Ay(1,2),Az(1,2);
        Vx<<0,0; Vy<<0,0; Vz<<0,0; Ax<<0,0; Ay<<0,0; Az<<0,0;
        MatrixXd ord(1,1); ord.fill(5);
        MatrixXd conx(3,2), cony(3,2), conz(3,2); conx<<Px,Vx,Ax; cony<<Py,Vy,Ay; conz<<Pz,Vz,Az;

        MatrixXd Xc = mj.Coefficient1(t, ord, conx, 0.1).transpose();
        MatrixXd Yc = mj.Coefficient1(t, ord, cony, 0.1).transpose();
        MatrixXd Zc = mj.Coefficient1(t, ord, conz, 0.1).transpose();

        Vector3d goal = r0 + Vector3d(dx,dy,dz);
        Matrix3d Rg   = H.rot(2, -100.0*M_PI/180.0, 3);

        for (double tt=0; tt<dur; tt+=T) {
            Vector3d V(
                mj.GetAccVelPos(Xc.row(0), tt, 0, 5)(0,1),
                mj.GetAccVelPos(Yc.row(0), tt, 0, 5)(0,1),
                mj.GetAccVelPos(Zc.row(0), tt, 0, 5)(0,1)
            );

            H.update_hand(q, V, goal, Rg);
            H.doQP(q);
            q = H.q_next;

            VectorXd qL = VectorXd::Zero(7);
            Vector3d head(0,0,0);
            publishMotorData(q, qL, head);

            ros::spinOnce();
            rate_.sleep();
        }
    };

    ROS_INFO("keyboard: W/S => Z+/Z- , D/A => Y+/Y- , E/Q => X+/X- , X => exit");
    bool exit_to_home = false;

    while (ros::ok()) {
        if ( (ros::WallTime::now() - last_input).toSec() > 60.0 ){
            exit_to_home = true;
            break;
        }

        fd_set set; FD_ZERO(&set); FD_SET(STDIN_FILENO, &set);
        timeval tv{0, 50000}; // 50ms
        int rv = select(STDIN_FILENO+1, &set, NULL, NULL, &tv);
        if (rv > 0 && FD_ISSET(STDIN_FILENO, &set)) {
            char ch=0;
            if (read(STDIN_FILENO, &ch, 1) == 1) {
                double dx=0, dy=0, dz=0; bool end=false, valid=false;
                switch (ch) {
                    case 'w': case 'W': dz = +0.10; valid=true; break; // Upward
                    case 's': case 'S': dz = -0.10; valid=true; break; // Downward
                    case 'd': case 'D': dy = +0.10; valid=true; break; // To the left
                    case 'a': case 'A': dy = -0.10; valid=true; break; // To the right
                    case 'e': case 'E': dx = +0.05; valid=true; break; // Forward
                    case 'q': case 'Q': dx = -0.05; valid=true; break; // Backward
                    case 'x': case 'X': end=true; break;
                    default: break;
                }
                if (end) { restoreTTY(); res.ok=true; res.message="exit"; return true; }
                if (valid) {
                    if (dx!=0) dx = std::max(std::min(dx,  MAX_DX),  -MAX_DX);
                    if (dy!=0) dy = std::max(std::min(dy,  MAX_DYZ), -MAX_DYZ);
                    if (dz!=0) dz = std::max(std::min(dz,  MAX_DYZ), -MAX_DYZ);
                    doRelative(dx,dy,dz, STEP_T);
                    last_input = ros::WallTime::now();
                }
            }
        }
        ros::spinOnce();
    }

    restoreTTY();
    res.ok = true; res.message = "timeout";
    q_right_state_ = q;
    return true;
}

bool HandManager::move_hand_general_handler(hand_planner::MoveHandGeneral::Request &req,
                                            hand_planner::MoveHandGeneral::Response &res)
{
    VectorXd q(7); 
    q << q_right_state_;
    const VectorXd q_init = q;

    auto pub = [&](const VectorXd& qr){
        VectorXd ql = VectorXd::Zero(7);
        publishMotorData(qr, ql, Vector3d(0,0,0));
    };

    const double T1 = 3.0, T2 = 3.0;
    bool user_exit = false;

    auto trim = [](std::string &s){
        while (!s.empty() && std::isspace((unsigned char)s.back())) s.pop_back();
        size_t i=0; while (i<s.size() && std::isspace((unsigned char)s[i])) ++i;
        s = s.substr(i);
    };

    for (const std::string& raw : req.commands) {
        std::string line = raw;
        trim(line);
        if (line.empty()) continue;
        if (line.size()==1 && (line[0]=='x' || line[0]=='X')) { user_exit = true; break; }

        std::istringstream iss(line);
        std::string mode; 
        if (!(iss >> mode)) continue;

        std::vector<double> vals; double tmp;
        while (iss >> tmp) vals.push_back(tmp);

        bool is_abs = (mode=="abs" || mode=="ABS" || mode=="Abs");
        bool is_rel = (mode=="rel" || mode=="REL" || mode=="Rel");
        if (!is_abs && !is_rel) continue;

        double mx=0,my=0,mz=0, gx=0,gy=0,gz=0, rx=0,ry=0,rz=0;
        if (is_abs) {
            if (vals.size() != 9) { res.ok=false; res.message="abs needs 9 values"; return true; }
            mx = vals[0]; my = vals[1]; mz = vals[2];
            gx = vals[3]; gy = vals[4]; gz = vals[5];
            rx = vals[6]; ry = vals[7]; rz = vals[8];
        } else { // rel
            if (vals.size() != 6) { res.ok=false; res.message="rel needs 6 values"; return true; }
            gx = vals[0]; gy = vals[1]; gz = vals[2];
            rx = vals[3]; ry = vals[4]; rz = vals[5];
        }

        hand_func_R.HO_FK_palm(q);
        Vector3d r0 = hand_func_R.r_palm;

        const double RX = rx * M_PI/180.0;
        const double RY = ry * M_PI/180.0;
        const double RZ = rz * M_PI/180.0;
        Matrix3d R_inc = hand_func_R.rot(2, RY, 3)
                       * hand_func_R.rot(1, RX, 3)
                       * hand_func_R.rot(3, RZ, 3);

        Vector3d r_goal, mid;
        Matrix3d R_goal;

        if (is_abs) {
            r_goal = Vector3d(gx, gy, gz);
            mid    = Vector3d(mx, my, mz);
            R_goal = R_inc;
        } else {
            r_goal = r0 + Vector3d(gx, gy, gz);

            mid = 0.5 * (r0 + r_goal);
            mid(1) -= 0.10;

            Matrix3d R0;
            if (hand_func_R.R_palm.rows()==3 && hand_func_R.R_palm.cols()==3) {
                R0 = hand_func_R.R_palm;
            } else {
                R0 = hand_func_R.R_palm.block<3,3>(0,0);
            }
            R_goal = R0 * R_inc;
        }

        ROS_INFO_STREAM("[move_hand_general rel/abs] r0="   << r0.transpose()
                         << " mid="   << mid.transpose()
                         << " goal="  << r_goal.transpose());

        if (!approachViaOneMid(hand_func_R, coef_generator, q, mid, r_goal, R_goal, T1, T2, T, pub)) {
            res.ok = false; res.message = "approach failed";
            if (req.go_home_on_finish) goHome(hand_func_R, q, q_init, 5.0, T, pub);
            return true;
        }
    }

    if (req.go_home_on_finish) {
        goHome(hand_func_R, q, q_init, 5.0, T, pub);
        res.ok = true; res.message = user_exit ? "exit-home" : "done-home";
    } else {
        res.ok = true; res.message = user_exit ? "exit" : "done";
    }

    q_right_state_ = q;
    return true;
}

bool HandManager::fingerControlService(hand_planner::FingerControl::Request &req, hand_planner::FingerControl::Response &res) {
    ROS_INFO("Received direct finger control request with hand selection: %s", req.hand_selection.c_str());
    
    try {
        // Convert hand selection to enum (default to RIGHT_HAND if not specified)
        HandSelection hand = (req.hand_selection.empty()) ? HandSelection::RIGHT_HAND : finger_control_->stringToHandSelection(req.hand_selection);
        
        // Convert request data to vectors
        std::vector<uint8_t> positions(req.target_positions.begin(), req.target_positions.end());
        std::vector<uint8_t> limits(req.pressure_limits.begin(), req.pressure_limits.end());
        global_finger_trigger = 1;
        
        bool success = finger_control_->setDirectControl(positions, limits, req.pid_kp, req.pid_ki, req.pid_kd, hand);
        
        if (success) {
            res.success = true;
            res.message = "Direct finger control parameters set successfully";
        } else {
            res.success = false;
            res.message = "Failed to set direct finger control parameters";
        }
        
    } catch (const std::exception& e) {
        res.success = false;
        res.message = "Exception in direct finger control: " + std::string(e.what());
    }
    global_finger_trigger = 0;
    return true;
}

bool HandManager::fingerScenarioService(hand_planner::FingerScenario::Request &req, hand_planner::FingerScenario::Response &res) {
    ROS_INFO("Received finger scenario request for scenario: %s with hand selection: %s", req.scenario_name.c_str(), req.hand_selection.c_str());
    
    try {
        // Convert hand selection to enum (default to RIGHT_HAND if not specified)
        HandSelection hand = (req.hand_selection.empty()) ? HandSelection::RIGHT_HAND : finger_control_->stringToHandSelection(req.hand_selection);
        global_finger_trigger = 1;
        
        if (finger_control_->executeScenario(req.scenario_name, hand)) {
            res.success = true;
            res.message = "Scenario executed successfully";
        } else {
            res.success = false;
            res.message = "Failed to execute scenario";
        }
        
    } catch (const std::exception& e) {
        res.success = false;
        res.message = "Exception in finger scenario: " + std::string(e.what());
    }
    global_finger_trigger = 0;
    return true;
}


// // --- Main Function ---
// int main(int argc, char **argv) {
//     ros::init(argc, argv, "hand_manager_node");
//     ros::NodeHandle n;
//     HandManager node_handler(&n);
//     ROS_INFO("Hand Manager Node is ready to receive service calls.");
//     ros::spin();
//     return 0;
// }