#include "RobotManager.h"

RobotManager::RobotManager(ros::NodeHandle *n) {
    // Instantiate the managers, passing the node handle so they can create their own services/pubs/subs
    hand_manager_ = std::make_unique<HandManager>(n);
    gait_manager_ = std::make_unique<GaitManager>(n);

    ROS_INFO("Robot Manager Node Initialized.");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_manager_node");
    ros::NodeHandle n;
    RobotManager robot_manager(&n);
    ROS_INFO("Robot Manager Node is running and ready for commands.");
    ros::spin();
    return 0;
}
