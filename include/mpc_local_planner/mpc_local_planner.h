#ifndef MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H
#define MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <mpc_ipopt/mpc.h>


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
        void publish_plan(const std::vector<mpc_ipopt::State> &plan);

        bool _initialised;
        unsigned int _i{0};

        tf2_ros::Buffer *_tf_buffer;
        costmap_2d::Costmap2DROS *_costmap;
        ros::Publisher _pub;

        std::unique_ptr<mpc_ipopt::MPC> _mpc;
        mpc_ipopt::MPC::Result mpc_result;

        double dt{0.1}, wheel_dist{1};

        std::pair<std::vector<double>, std::vector<double> > _plan;

        std::pair<double, double> _vel;

        const size_t poly_order = 3;

        // void reconfigureCB(DWAPlannerConfig &config, uint32_t level);
    };
}

#endif //MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H
