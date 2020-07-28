#ifndef MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H
#define MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H

#include <mpc_local_planner/helper.h>

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <mpc_lib/mpc.h>

#include <vector>

namespace mpc_local_planner {

    class MPC_Local_Planner : public nav_core::BaseLocalPlanner {

    public:
        void initialize(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap) override;

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;

        bool isGoalReached() override;

        ~MPC_Local_Planner() override;

        MPC_Local_Planner();

        [[nodiscard]] bool isInitialized() const {
            return _initialised;
        }

    private:
        bool get_trans(double &x, double &y, double &yaw) const;

        void publish_plan(const std::vector<mpc_lib::State> &plan);

        [[nodiscard]] std::vector<std::vector<double>> getObstacles() const;

        bool _initialised;

        unsigned int _i{0};
        // ROS Things
        tf2_ros::Buffer *_tf_buffer{nullptr};
        costmap_2d::Costmap2DROS *_costmap{nullptr};
        std::unique_ptr<ros::Publisher> _path_pub, _global_plan_pub;
        std::unique_ptr<helper::PCPublisher> _pc_pub;

        std::array<std::unique_ptr<ros::Publisher>, 5> _poly_pubs;
        // mpc_lib things
        std::unique_ptr<mpc_lib::MPC> _mpc;

        mpc_lib::MPC::Result mpc_result;
        // Params
        double dt{0.1}, mpc_dt{0.1}, wheel_dist{1};

        const size_t poly_order = 3;

        /*
         * dt vs mpc_dt:
         *
         * We pass mpc the state that the robot is expected to have when the velocity calculation is finished
         * `dt` is used to estimate that. i.e dt === time
         *
         * mpc_dt is the timeinterval mpc uses to forward simulate.
         * We use this to calculate the velocity from the calculated acceleration
         * TODO: Should mpc directly give velocity?
         */

        // State
        std::pair<std::vector<double>, std::vector<double>> _plan;

        std::pair<double, double> _vel{0, 0};

        // void reconfigureCB(DWAPlannerConfig &config, uint32_t level);
    };
}

#endif //MPC_LOCAL_PLANNER_MPC_LOCAL_PLANNER_H
