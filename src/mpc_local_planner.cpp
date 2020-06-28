#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"

#include "../include/mpc_local_planner/mpc_local_planner.h"
//#include "mpc_local_planner/mpc_local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MPC_Local_Planner, nav_core::BaseLocalPlanner)


// implement MPC_Local_Planner
using namespace mpc_local_planner;

MPC_Local_Planner::MPC_Local_Planner() {
    ROS_INFO("Constructing Planner");
}

MPC_Local_Planner::~MPC_Local_Planner() {
    ROS_INFO("Destroying Planner");
}


void MPC_Local_Planner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) {
    ROS_INFO("Initing Planner");
}

bool MPC_Local_Planner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
    ROS_INFO("Setting Plan");
}

bool MPC_Local_Planner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
    ROS_INFO("Compute vel");

}

bool MPC_Local_Planner::isGoalReached() {
    ROS_INFO("Checking if goal reached");
}

#pragma clang diagnostic pop