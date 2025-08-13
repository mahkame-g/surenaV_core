#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "ros/ros.h"
#include <memory>

// Include the managers from the two library packages
#include "HandManager.h"
#include "GaitManager.h"

class RobotManager {
public:
    RobotManager(ros::NodeHandle *n);

private:
    // Pointers to the specialized managers
    std::unique_ptr<HandManager> hand_manager_;
    std::unique_ptr<GaitManager> gait_manager_;
};

#endif // ROBOT_MANAGER_H