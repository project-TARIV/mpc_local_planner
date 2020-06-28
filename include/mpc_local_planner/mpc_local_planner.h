#ifndef MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H
#define MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <mpc_lib/dDrive_MPC.h>

namespace mpc_local_planner {
    class MPC_Local_Planner : public nav_core::BaseLocalPlanner {
    public:
        void initialize(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();

        ~MPC_Local_Planner();

        MPC_Local_Planner();

        bool isInitialized() {
            return _initialised;
        }

    private:
        bool _initialised;

        tf2_ros::Buffer *_tf_buffer;
        costmap_2d::Costmap2DROS *_costmap;

        MPC _mpc;

        // void reconfigureCB(DWAPlannerConfig &config, uint32_t level);
    };
}

#endif //MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H
