#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"

#include "../include/mpc_local_planner/mpc_local_planner.h"
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Core>
#include <mpc_lib/utils.h>


//#include "mpc_local_planner/mpc_local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MPC_Local_Planner, nav_core::BaseLocalPlanner)


// implement MPC_Local_Planner
using namespace mpc_local_planner;

MPC_Local_Planner::MPC_Local_Planner() : _initialised{false}, _tf_buffer(nullptr), _costmap(nullptr), _mpc(),
                                         _last_vel() {
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

        // Todo get velocity, dont assume 0
        _throttle = 0;
        _last_vel = {};
        _last_called = ros::Time::now();

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
    _plan.first.clear();
    _plan.first.reserve(plan.size());
    _plan.second.clear();
    _plan.second.reserve(plan.size());
    for (const auto &pose : plan) {
        _plan.first.push_back(pose.pose.position.x);
        _plan.second.push_back(pose.pose.position.y);
    }

    // Temp
    //_last_called = ros::Time::now();

    ROS_INFO("Setting Plan, of length %li", plan.size());

    return true;
}

bool MPC_Local_Planner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }

    // Check whether we should remove points from path..


    ROS_INFO("Starting to compute velocity");
    auto time = ros::Time::now();
    const double dt = (time - _last_called).toSec();

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
    //ROS_INFO("xyw: %fl, %fl, %fl", x, y, yaw);
    ROS_INFO("Got frame");

    const size_t num_pts = 20;
    assert(_plan.first.size() >= num_pts);
    std::vector<double> ptsx, ptsy;
    ptsx.resize(num_pts);
    ptsy.resize(num_pts);
    for (int i = 0; i < num_pts; i++) {
        double shift_x = _plan.first[i] - x;
        double shift_y = _plan.second[i] - y;
        ptsx[i] = shift_x * cos(-yaw) - shift_y * sin(-yaw);
        ptsy[i] = shift_x * sin(-yaw) + shift_y * cos(-yaw);
        //std::cout << _plan.first[i] << " " << _plan.second[i] << " " << ptsx[i] << " " << ptsy[i] << std::endl;
    }
    Eigen::VectorXd coeffs = polyfit(Eigen::Map<Eigen::VectorXd>(ptsx.data(), std::min(num_pts, ptsx.size())),
                                     Eigen::Map<Eigen::VectorXd>(ptsy.data(), std::min(num_pts, ptsy.size())),
                                     3);
    ROS_INFO("Got poly");
    using std::cout;
    cout << "poly" << coeffs.size() << std::endl << std::fixed;
    for (int i = 0; i < coeffs.size(); i++) {
        cout << " + " << coeffs(i) << "*x^" << i << " ";
    }
    cout << std::endl << std::scientific;

    double cte = coeffs(0);// polyeval(coeffs, 0);
    double etheta = -atan(coeffs[1]);
    std::cout << cte << " " << etheta * 180 / pi() << std::endl;

    State s{};
    std::cout << dt << std::endl;
    const double v = _last_vel.linear.x;
    const double &omega = _last_vel.angular.z;
    //double dt = _mpc.params.dt;
    s.x = v * dt;
    s.y = 0;
    s.theta = omega * dt;
    s.v = v + _throttle * dt;
    s.cte = cte + v * sin(etheta) * dt; // Change in te required , i.e position?
    s.etheta = etheta - s.theta; // Change in angle required
    _mpc.params.dt = dt;
    std::cout << s.x << " " << s.y << " " << s.theta * 180 / pi() << " " << s.v << " " << s.cte << " "
              << s.etheta * 180 / pi() << std::endl;
    // time to solve !
    std::vector<double> mpc_solns;
    if (!_mpc.Solve(s, coeffs, mpc_solns))
        return false;

    ROS_INFO("Got solution");
    cmd_vel.angular.z = mpc_solns[0];
    _throttle = mpc_solns[1];
    cmd_vel.linear.x = v + _throttle * dt;

    // Update throttle
    _last_vel = cmd_vel;
    // Update last time
    _last_called = time;
    ROS_INFO("Compute vel");
    return true;
}

bool MPC_Local_Planner::isGoalReached() {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }

    //IDK

    ROS_INFO("Checking if goal reached");
    return _plan.first.empty();
}

#pragma clang diagnostic pop