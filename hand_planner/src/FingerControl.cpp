#include "FingerControl.h"

uint8_t global_finger_trigger = 0;

FingerControl::FingerControl(ros::NodeHandle* nh) : nh_(nh) {
    // Initialize ROS communication
    finger_data_pub_ = nh_->advertise<std_msgs::Int32MultiArray>("fingerdata/qc", 100);
    
    // Load scenarios from JSON file
    std::string scenarios_path = ros::package::getPath("hand_planner") + "/config/finger_scenarios.json";
    std::ifstream fr(scenarios_path);
    json scenarios_json = json::parse(fr);
    
    for (auto& [name, scenario_data] : scenarios_json["scenarios"].items()) {
        FingerScenario scenario;
        scenario.name = name;
        scenario.target_positions = scenario_data["target_positions"].get<std::vector<uint8_t>>();
        scenario.target_speeds = scenario_data["target_speeds"].get<std::vector<uint8_t>>();
        scenario.pressure_limits = scenario_data["pressure_limits"].get<std::vector<uint8_t>>();
        scenario.pid_kp = scenario_data["pid_kp"].get<uint8_t>();
        scenario.pid_ki = scenario_data["pid_ki"].get<uint8_t>();
        scenario.pid_kd = scenario_data["pid_kd"].get<uint8_t>();
        
        scenarios_[name] = scenario;
    }
    ROS_INFO("Finger Control initialized");
}

bool FingerControl::executeScenario(const std::string& name, HandSelection hand) {
    std::lock_guard<std::mutex> lock(scenarios_mutex_);
    
    auto it = scenarios_.find(name);
    if (it == scenarios_.end()) {
        ROS_ERROR("Scenario '%s' not found", name.c_str());
        return false;
    }
    
    const FingerScenario& scenario = it->second;
    std::string hand_str = (hand == HandSelection::RIGHT_HAND) ? "right" : (hand == HandSelection::LEFT_HAND) ? "left" : "both";
    ROS_INFO("Executing scenario: %s for %s hand(s)", scenario.name.c_str(), hand_str.c_str());
    
    // Create message with 23 elements: 6 positions + 6 speeds + 6 pressure + 3 PID + 1 right trigger + 1 left trigger
    std_msgs::Int32MultiArray msg;
    msg.data.resize(23);
    for (int i = 0; i < 23; ++i) {
        msg.data[i] = 0;
    }
    
    // Fill positions (indices 0-5)
    for (int i = 0; i < 6; ++i) {
        msg.data[i] = scenario.target_positions[i];
    }
    
    // Fill speeds (indices 6-11)
    for (int i = 0; i < 6; ++i) {
        msg.data[i + 6] = scenario.target_speeds[i];
    }
    
    // Fill pressure limits (indices 12-17)
    for (int i = 0; i < 6; ++i) {
        msg.data[i + 12] = scenario.pressure_limits[i];
    }
    
    // Fill PID values (indices 18-20)
    msg.data[18] = scenario.pid_kp;
    msg.data[19] = scenario.pid_ki;
    msg.data[20] = scenario.pid_kd;
    
    // Set triggers based on hand selection (indices 21-22)
    if (hand == HandSelection::RIGHT_HAND || hand == HandSelection::BOTH_HANDS) {
        msg.data[21] = global_finger_trigger;  // Right hand trigger
    }
    if (hand == HandSelection::LEFT_HAND || hand == HandSelection::BOTH_HANDS) {
        msg.data[22] = global_finger_trigger;  // Left hand trigger
    }
    
    // Publish message for 2 seconds
    ros::Rate rate(200);
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 2.0) {
        finger_data_pub_.publish(msg);
        rate.sleep();
    }
    
    ROS_INFO("Scenario '%s' executed successfully for %s hand(s)", scenario.name.c_str(), hand_str.c_str());
    return true;
}

bool FingerControl::setDirectControl(const std::vector<uint8_t>& positions,
                                    const std::vector<uint8_t>& speeds,
                                    const std::vector<uint8_t>& limits,
                                    uint8_t kp, uint8_t ki, uint8_t kd,
                                    HandSelection hand) {
    ROS_INFO("Setting direct finger control parameters");
    
    // Validate input sizes
    if (positions.size() != 6) {
        ROS_ERROR("Invalid positions size: %zu (expected: 6)", positions.size());
        return false;
    }
    if (speeds.size() != 6) {
        ROS_ERROR("Invalid speeds size: %zu (expected: 6)", speeds.size());
        return false;
    }
    if (limits.size() != 6) {
        ROS_ERROR("Invalid pressure limits size: %zu (expected: 6)", limits.size());
        return false;
    }
    
    std::string hand_str = (hand == HandSelection::RIGHT_HAND) ? "right" : (hand == HandSelection::LEFT_HAND) ? "left" : "both";
    ROS_INFO("Executing direct control for %s hand(s)", hand_str.c_str());
    
    // Create message with 23 elements: 6 positions + 6 speeds + 6 pressure + 3 PID + 1 right trigger + 1 left trigger
    std_msgs::Int32MultiArray msg;
    msg.data.resize(23);
    for (int i = 0; i < 23; ++i) {
        msg.data[i] = 0;
    }
    
    // Fill positions (indices 0-5)
    for (int i = 0; i < 6; ++i) {
        msg.data[i] = positions[i];
    }
    
    // Fill speeds (indices 6-11)
    for (int i = 0; i < 6; ++i) {
        msg.data[i + 6] = speeds[i];
    }
    
    // Fill pressure limits (indices 12-17)
    for (int i = 0; i < 6; ++i) {
        msg.data[i + 12] = limits[i];
    }
    
    // Fill PID values (indices 18-20)
    msg.data[18] = kp;
    msg.data[19] = ki;
    msg.data[20] = kd;
    
    // Set triggers based on hand selection (indices 21-22)
    if (hand == HandSelection::RIGHT_HAND || hand == HandSelection::BOTH_HANDS) {
        msg.data[21] = global_finger_trigger;  // Right hand trigger
    }
    if (hand == HandSelection::LEFT_HAND || hand == HandSelection::BOTH_HANDS) {
        msg.data[22] = global_finger_trigger;  // Left hand trigger
    }
    
    // Publish message for 2 seconds
    ros::Rate rate(200);
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 2.0) {
        finger_data_pub_.publish(msg);
        rate.sleep();
    }
    
    ROS_INFO("Direct finger control executed successfully for %s hand(s)", hand_str.c_str());
    return true;
}

bool FingerControl::moveMotor(uint8_t motor_id, uint8_t position, uint8_t speed, HandSelection hand) {
    if (motor_id >= 6) {
        ROS_ERROR("Invalid motor ID: %d (must be 0-5)", motor_id);
        return false;
    }
    
    std::string hand_str = (hand == HandSelection::RIGHT_HAND) ? "right" : (hand == HandSelection::LEFT_HAND) ? "left" : "both";
    ROS_INFO("Moving motor %d to position %d with speed %d (%s hand(s))", motor_id, position, speed, hand_str.c_str());
    
    // Create message with 23 elements: 6 positions + 6 speeds + 6 pressure + 3 PID + 1 right trigger + 1 left trigger
    std_msgs::Int32MultiArray msg;
    msg.data.resize(23);
    for (int i = 0; i < 23; ++i) {
        msg.data[i] = 0;
    }
    
    // Set motor data in positions (indices 0-5) - only set the specific motor
    msg.data[motor_id] = position;
    
    // Set speed in speeds (indices 6-11) - only set the specific motor
    msg.data[motor_id + 6] = speed;
    
    // Set triggers based on hand selection (indices 21-22)
    if (hand == HandSelection::RIGHT_HAND || hand == HandSelection::BOTH_HANDS) {
        msg.data[21] = global_finger_trigger;  // Right hand trigger
    }
    if (hand == HandSelection::LEFT_HAND || hand == HandSelection::BOTH_HANDS) {
        msg.data[22] = global_finger_trigger;  // Left hand trigger
    }
    
    // Publish message for 2 seconds
    ros::Rate rate(200);
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 2.0) {
        finger_data_pub_.publish(msg);
        rate.sleep();
    }
    
    ROS_INFO("Motor %d movement completed for %s hand(s)", motor_id, hand_str.c_str());
    return true;
}

HandSelection FingerControl::stringToHandSelection(const std::string& hand_str) {
    // Convert to lowercase for case-insensitive comparison
    std::string lower_hand = hand_str;
    std::transform(lower_hand.begin(), lower_hand.end(), lower_hand.begin(), ::tolower);
    
    if (lower_hand == "right") {
        return HandSelection::RIGHT_HAND;
    } else if (lower_hand == "left") {
        return HandSelection::LEFT_HAND;
    } else if (lower_hand == "both") {
        return HandSelection::BOTH_HANDS;
    } else {
        ROS_WARN("Invalid hand selection: '%s', defaulting to RIGHT_HAND", hand_str.c_str());
        return HandSelection::RIGHT_HAND;  // Default to right hand
    }
}