#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"

#include "../include/mpc_local_planner/mpc_local_planner.h"
//#include "mpc_local_planner/mpc_local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MPC_Local_Planner, nav_core::BaseLocalPlanner)


// implement MPC_Local_Planner
using namespace mpc_local_planner;

MPC_Local_Planner::MPC_Local_Planner() : _initialised{false}, _tf_buffer(nullptr), _costmap(nullptr), _mpc() {
    ROS_INFO("Constructing Planner");
}

MPC_Local_Planner::~MPC_Local_Planner() {
    ROS_INFO("Destroying Planner");

}


void MPC_Local_Planner::initialize(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap) {
    ROS_INFO("Initing Planner");
    if (!isInitialized()) {
        _tf_buffer = tf_buffer;
        _costmap = costmap;

        ros::NodeHandle private_nh("~/" + name);

//        _dsrv_ = new dynamic_reconfigure::Server<mpc_local_planner_config>(private_nh);
//        dynamic_reconfigure::Server<mpc_local_planner_config>::CallbackType cb = boost::bind(&MPC_Local_Planner::reconfigureCB, this, _1, _2);
//        _dsrv->setCallback(cb);

        // TODO: Do this better
        private_nh.getParam("controller/timeSteps", _mpc.params.N);
        private_nh.getParam("controller/sampleTime", _mpc.params.dt);

        private_nh.getParam("controller/reference/velocity", _mpc.params.ref_v);
        private_nh.getParam("controller/reference/crossTrackError", _mpc.params.ref_cte);
        private_nh.getParam("controller/reference/orientationError", _mpc.params.ref_etheta);

        private_nh.getParam("controller/maxBounds/maxOmega", _mpc.params.MAX_OMEGA);
        private_nh.getParam("controller/maxBounds/maxThrottle", _mpc.params.MAX_THROTTLE);

        private_nh.getParam("controller/weights/w_cte", _mpc.params.W_CTE);
        private_nh.getParam("controller/weights/w_etheta", _mpc.params.W_ETHETA);
        private_nh.getParam("controller/weights/w_vel", _mpc.params.W_VEL);
        private_nh.getParam("controller/weights/w_omega", _mpc.params.W_OMEGA);
        private_nh.getParam("controller/weights/w_acc", _mpc.params.W_ACC);
        private_nh.getParam("controller/weights/w_omega_d", _mpc.params.W_OMEGA_D);
        private_nh.getParam("controller/weights/w_acc_d", _mpc.params.W_ACC_D);


    } else {
        ROS_WARN("Already initialised, doing nothing.");
    }
}

bool MPC_Local_Planner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }
    // Convert path to relevent frame and save it locally

    ROS_INFO("Setting Plan, %li", plan.size());

    return true;
}

bool MPC_Local_Planner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }

    // mpc.solve

    ROS_INFO("Compute vel");
    return false;

}

bool MPC_Local_Planner::isGoalReached() {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }

    //IDK

    ROS_INFO("Checking if goal reached");
    return false;
}

#pragma clang diagnostic pop