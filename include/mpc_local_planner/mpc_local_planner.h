#ifndef MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H
#define MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

namespace mpc_local_planner {
    class MPC_Local_Planner : public nav_core::BaseLocalPlanner {
    public:
        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
        
        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();

        ~MPC_Local_Planner();

    //protected:
        MPC_Local_Planner();

    private:

    };
}

#endif //MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H
