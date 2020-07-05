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
        void publish_plan(const std::vector<std::pair<double,double>> & plan);

        bool _initialised;
        unsigned int _i;

        tf2_ros::Buffer *_tf_buffer;
        costmap_2d::Costmap2DROS *_costmap;
        ros::Publisher _pub;

        mpc_lib::MPC _mpc;


        std::pair<std::vector<double>, std::vector<double> > _plan;

        geometry_msgs::Twist _last_vel;
        double _accel{0};
        ros::Time _last_called; // or should i just use controller frequency?

        // void reconfigureCB(DWAPlannerConfig &config, uint32_t level);
    };
}

#endif //MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H
