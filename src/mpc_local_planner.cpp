#include "../include/mpc_local_planner/mpc_local_planner.h"
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <mpc_lib/utils.h>
#include <nav_msgs/Path.h>


#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MPC_Local_Planner, nav_core::BaseLocalPlanner)


// implement MPC_Local_Planner
using namespace mpc_local_planner;

MPC_Local_Planner::MPC_Local_Planner() : _initialised{false}, _tf_buffer(nullptr), _costmap(nullptr), _mpc(),
                                         _last_vel(), _pub() {
    ROS_INFO("Constructed Planner");
}

MPC_Local_Planner::~MPC_Local_Planner() {
    ROS_INFO("Destroying Planner");
}


void MPC_Local_Planner::initialize(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap) {
    if (!isInitialized()) {
        ROS_INFO("Initing Planner");

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

        // Todo get velocity, dont assume 0
        _accel = 0;
        _last_vel = {};
        _last_called = ros::Time::now();

        _pub = private_nh.advertise<nav_msgs::Path>("path", 2);
        _initialised = true;
    } else {
        ROS_WARN("Already initialised, doing nothing.");
    }
}

bool MPC_Local_Planner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }
    // Convert plan to a pair of vectors

    _plan.first.clear();
    _plan.first.reserve(plan.size());
    _plan.second.clear();
    _plan.second.reserve(plan.size());
    for (const auto &pose : plan) {
        _plan.first.push_back(pose.pose.position.x);
        _plan.second.push_back(pose.pose.position.y);
    }


    ROS_INFO("Setting Plan, of length %li", plan.size());

    return true;
}

bool MPC_Local_Planner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }

    // TODO: Check whether we should remove points from path..



    auto time = ros::Time::now();
    const double dt = (time - _last_called).toSec();
    std::cout << 1 / dt << std::endl;


    // Get transform to base_frame from global frame
    double x, y, yaw;
    try {
        auto trans = _tf_buffer->lookupTransform(_costmap->getGlobalFrameID(), _costmap->getBaseFrameID(),
                                                 ros::Time(0)).transform;
        x = trans.translation.x;
        y = trans.translation.y;
        double _roll, _pitch;
        tf2::Matrix3x3(tf2::Quaternion{trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w}).getRPY(
                _roll, _pitch, yaw);
    } catch (tf2::TransformException &e) {
        ROS_ERROR("Can't get transform from %s to %s.",
                  _costmap->getGlobalFrameID().c_str(), _costmap->getBaseFrameID().c_str());
        return false;
    }

    // Get transform the points to be fitted to base frame
    // TODO: Transform polynomial instead?
    const size_t num_pts = std::min(20ul, _plan.first.size());
    assert(_plan.first.size() >= num_pts);
    std::vector<double> plan_x_trans, plan_y_trans;
    plan_x_trans.resize(num_pts);
    plan_y_trans.resize(num_pts);
    for (int i = 0; i < num_pts; i++) {
        double shift_x = _plan.first[i] - x;
        double shift_y = _plan.second[i] - y;
        plan_x_trans[i] = shift_x * cos(-yaw) - shift_y * sin(-yaw);
        plan_y_trans[i] = shift_x * sin(-yaw) + shift_y * cos(-yaw);
        //std::cout << _plan.first[i] << " " << _plan.second[i] << " " << plan_x_trans[i] << " " << plan_y_trans[i] << std::endl;
    }

    // Fit transformed plan to a polynomial
    Eigen::VectorXd coeffs = mpc_lib::polyfit(
            Eigen::Map<Eigen::VectorXd>(plan_x_trans.data(), num_pts), // std::min(num_pts, plan_x_trans.size())),
            Eigen::Map<Eigen::VectorXd>(plan_y_trans.data(), num_pts), //std::min(num_pts, plan_y_trans.size())),
            3);


/*
    // Print polynomial in format easy to chuck into desmos
    using std::cout;
    cout << "poly" << coeffs.size() << std::endl << std::fixed;
    for (int i = 0; i < coeffs.size(); i++) {
        cout << " + " << coeffs(i) << "*x^" << i << " ";
    }
    cout << std::endl << std::scientific;
*/


    // cte: 'Cross track error'
    double cte = coeffs(0); // polyeval(coeffs, 0);
    // etheta: 'Theta error' TODO: doesnt consider directionaity of path. i.e robot cannot uturn
    double etheta = -atan(coeffs[1]);
    std::cout << cte << " " << etheta * 180 / mpc_lib::pi() << std::endl;


    mpc_lib::State s{};
    const double &v = _last_vel.linear.x;
    const double &omega = _last_vel.angular.z;

    // TODO: Whats going on here?
    s.x = v * dt;
    s.y = 0;
    s.theta = omega * dt;
    s.v = v + _accel * dt;
    s.cte = cte + v * sin(etheta) * dt; // Change in te required , i.e position?
    s.etheta = etheta - s.theta; // Change in angle required
/*
    s.x = 0;
    s.y = 0;
    s.theta = 0;
    s.v = _last_vel.linear.x;
    s.cte = cte;
    s.etheta = etheta;*/


    std::vector<double> mpc_solns;
    mpc_lib::Result mpc_result;
    /*if (!_mpc.solve(s, coeffs, mpc_result))
        return false;*/
    if (!_mpc.solve(s, coeffs, mpc_result, true))
        return false;


    ROS_INFO("Cost: %f", mpc_result.cost);

    cmd_vel.angular.z = mpc_result.omega;
    _accel = mpc_result.acceleration;
    cmd_vel.linear.x = v + _accel * dt;

    // Update velocity and time
    _last_vel = cmd_vel;
    _last_called = time;

    _i++;
    _i %= 20;
    if (_i == 0)
        publish_plan(mpc_result.plan);

    return true;
}

bool MPC_Local_Planner::isGoalReached() {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }

    ROS_INFO("Checking if goal reached");
    return _plan.first.empty();
}

void MPC_Local_Planner::publish_plan(const std::vector<std::pair<double, double>> &plan) {
    /*if (!_pub)
        return;*/
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = _costmap->getBaseFrameID();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = _costmap->getBaseFrameID();
    pose.pose.orientation.w = 1;

    for (const auto &point : plan) {
        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;

        path.poses.emplace_back(pose);
    }

    _pub.publish(path);
}
