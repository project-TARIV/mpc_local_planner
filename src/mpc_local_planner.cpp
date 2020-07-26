#include "../include/mpc_local_planner/mpc_local_planner.h"
#include <tf2/utils.h>
#include <nav_msgs/Path.h>
#include <virat_msgs/Polynomial.h>
#include "polyfit.h"
#include "shadow_casting.h"

#include <chrono>
#include <algorithm>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MPC_Local_Planner, nav_core::BaseLocalPlanner)


// simple signum from math
// Used for directionality
inline double signum(double x) {
    return (x > 0) - (x < 0);
}



// implement MPC_Local_Planner
using namespace mpc_local_planner;

MPC_Local_Planner::MPC_Local_Planner() : _initialised{false} {
    ROS_INFO("Constructed Planner");
}

MPC_Local_Planner::~MPC_Local_Planner() {
    ROS_INFO("Destroying Planner");
}


void MPC_Local_Planner::initialize(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap) {
    if (!isInitialized()) {
        ROS_INFO("Initializing Planner %s", name.data());

        _tf_buffer = tf_buffer;
        _costmap = costmap;

        ros::NodeHandle private_nh("~/" + name);
        // TODO: Warn on missing param
        mpc_lib::Params p{};

        /* Get Parameter values */ {
            private_nh.getParam("wheel_dist", p.wheel_dist);
            wheel_dist = p.wheel_dist;

            double tmp;
            private_nh.getParam("timeSteps", tmp);
            assert(tmp > 0);
            p.forward.steps = (size_t) tmp;

            private_nh.getParam("frequency", p.forward.frequency);

            private_nh.getParam("reference/velocity", p.v_ref);

            private_nh.getParam("limits/vel/low", p.limits.vel.low);
            private_nh.getParam("limits/vel/high", p.limits.vel.high);
            private_nh.getParam("limits/acc/low", p.limits.acc.low);
            private_nh.getParam("limits/acc/high", p.limits.acc.high);

            private_nh.getParam("weights/cte", p.wt.cte);
            private_nh.getParam("weights/etheta", p.wt.etheta);
            private_nh.getParam("weights/vel", p.wt.vel);
            private_nh.getParam("weights/omega", p.wt.omega);
            private_nh.getParam("weights/acc", p.wt.acc);
        }

        _mpc = std::make_unique<mpc_lib::MPC>(p);


        // dt should be the length of one iteration of getCmdVel
        dt = 130.0e-3; // 130 ms very big approx Get this from profiling
        mpc_dt = 1.0 / p.forward.frequency;

        // Todo get velocity,ca dont assume 0
        _vel = {0, 0};

        _path_pub = std::make_unique<ros::Publisher>(private_nh.advertise<nav_msgs::Path>("path", 2));
        _global_plan_pub = std::make_unique<ros::Publisher>(
                private_nh.advertise<virat_msgs::Polynomial>("global_plan", 2));
        for (size_t i = 0; i < _poly_pubs.size(); i++) {
            _poly_pubs[i] = std::make_unique<ros::Publisher>(
                    private_nh.advertise<virat_msgs::Polynomial>("polys/poly" + std::to_string(i), 2));
        }

        _pc_pub = std::make_unique<helper::PCPublisher>(
                private_nh.advertise<sensor_msgs::PointCloud2>("obstacle_points", 2));

        _initialised = true;
    } else {
        ROS_WARN("Already initialised, doing nothing.");
    }
}

// Convert plan to a pair of vectors with x and y
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
        // TODO: Fit theta values from plan into a polynomial and use that !
    }


    ROS_INFO("Setting Plan, of length %li", plan.size());

    return true;
}

bool MPC_Local_Planner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }
//    const auto start = std::chrono::high_resolution_clock::now(); // < 1 ms used outside of mpc

    // TODO: Can we use transform from goal_reached ?
    // Get transform to base_frame from global frame
    double x, y, yaw;
    if (!get_trans(x, y, yaw))
        return false;

    auto &px = _plan.first;
    auto &py = _plan.second;


    /* Check if goal reached */{
        size_t i = 0;

        for (; i < px.size() - 1; i++) {
            // This is dot product of vector A-P and B-P
            // Where P is current robot position,  A is _plan[i] and B is _plan[i+1]
            if ((x - px[i]) * (px[i + 1] - px[i]) + (y - py[i]) * (py[i + 1] - py[i]) < 0) {
                break; // i.e this point not crossed yet.
            }
        }
        if (i > 0) {
            px.erase(px.begin(), px.begin() + i);
            py.erase(py.begin(), py.begin() + i);
            ROS_INFO("Removed %lu points, now %lu", i, px.size());
        }
        // TODO: Reverse the vector so deletions from path dont require whole path to be copied. Switch to list if this matters
    }

    if (px.size() <= 10 /* poly_order*/) {
        px.clear();
        py.clear();
        // TODO: allow to coast through waypoints.
        _vel = {0, 0};
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        ROS_INFO("Goal Reached");
        return true;
    }


    // Get transform the points to be fitted to base frame
    const size_t num_pts = std::min(40ul, px.size());
    assert(num_pts > poly_order);
    std::vector<double> plan_x_trans, plan_y_trans;
    plan_x_trans.resize(num_pts);
    plan_y_trans.resize(num_pts);
    for (int i = 0; i < num_pts; i++) {
        const double shift_x = px[i] - x;
        const double shift_y = py[i] - y;
        plan_x_trans[i] = shift_x * cos(-yaw) - shift_y * sin(-yaw);
        plan_y_trans[i] = shift_x * sin(-yaw) + shift_y * cos(-yaw);
        //std::cout << px[i] << " " << py[i] << " " << plan_x_trans[i] << " " << plan_y_trans[i] << std::endl;
    }

    // Fit transformed plan to a polynomial
    Eigen::VectorXd coeffs = mpc_local_planner::polyfit(
            Eigen::Map<Eigen::VectorXd>(plan_x_trans.data(), num_pts), // std::min(num_pts, plan_x_trans.size())),
            Eigen::Map<Eigen::VectorXd>(plan_y_trans.data(), num_pts), //std::min(num_pts, plan_y_trans.size())),
            3);

    // Set global plan coefficients
    _mpc->global_plan.clear();
    _mpc->global_plan.resize(coeffs.size());
    for (size_t i = 0; i < coeffs.size(); i++) {
        _mpc->global_plan.push_back(coeffs[i]);
    }
    // Print polynomial in format easy to copy into desmos
    /*using std::cout;
    cout << "poly" << coeffs.size() << std::endl << std::fixed;
    for (int i = 0; i < coeffs.size(); i++) {
        cout << " + " << coeffs(i) << "*x^" << i << " ";
    }
    cout << std::endl << std::scientific;
     //*/

    virat_msgs::Polynomial p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = _costmap->getGlobalFrameID();
    p.coeffs = {coeffs[0], coeffs[1], coeffs[2]};
    _global_plan_pub->publish(p);
    std::cout << "Published!" << std::endl;

    // TODO: Fit theta values from plan into a polynomial and use that !


    // Pass estimated state at the end
    _mpc->state.x = (_vel.first + _vel.second) * dt / 2 * 1; // 1 <= cos(theta)
    _mpc->state.y = (_vel.first + _vel.second) * dt / 2 * 0; // 1 <= sin(theta)
    _mpc->state.theta = (_vel.first - _vel.second) * dt / wheel_dist;
    _mpc->state.v_r = _vel.first;
    _mpc->state.v_l = _vel.second;



    /*unsigned int mx, my;
    _costmap->getCostmap()->worldToMap(x, y, mx, my);
    std::cout << mx << " " << my << " " << _costmap->getCostmap()->getSizeInCellsX() << " "
              << _costmap->getCostmap()->getSizeInCellsY() << std::endl;*/


    _mpc->directionality = signum(plan_x_trans[0]);

    std::vector<std::vector<double>> polys = getObstacles();


    p.header.stamp = ros::Time::now();
    p.header.frame_id = _costmap->getGlobalFrameID();
    size_t i = 0;
    for (const auto &poly: polys) {
        p.coeffs = {poly[0], poly[1], poly[2]};
        _poly_pubs[i]->publish(p);
        i++;
        if (i > _poly_pubs.size())
            break;
    }

    // Clear any old obstacles
    while (i < _poly_pubs.size()) {
        p.coeffs = {};
        _poly_pubs[i]->publish(p);
        i++;
    }

    if (!_mpc->solve(mpc_result, _i == 0)) {
        std::cout << "IPOPT Failed: " << mpc_lib::MPC::error_string.at(mpc_result.status) << std::endl;
        return false;
    }

    // MPC finds acc based on its own dt, use that.
    _vel.first += mpc_result.acc.first * mpc_dt;
    _vel.second += mpc_result.acc.second * mpc_dt;

    cmd_vel.angular.z = (_vel.first - _vel.second) / wheel_dist;
    cmd_vel.linear.x = (_vel.first + _vel.second) / 2;

    if (_i == 0)
        publish_plan(mpc_result.path);
    /*_i++;
    _i %= 20;*/
    _i = 0;

//    std::cout << "Iter took " << std::chrono::duration_cast<std::chrono::milliseconds>(
//            std::chrono::high_resolution_clock::now() - start).count() << "ms." << std::endl; // < 1 ms used outside of mpc
    return true;
}

std::vector<std::vector<double>> MPC_Local_Planner::getObstacles() const {
    double x, y, yaw;
    if (!get_trans(x, y, yaw)) return {};

    std::vector<std::vector<double>> polynomials;

    auto cp = _costmap->getCostmap();

    // Get robot location in costmap
    // i.e (c)enter of the shadow_casting
    unsigned int cx, cy;
    cp->worldToMap(x, y, cx, cy);

    // The costmap co-ordinates of obstacle_points
    std::vector<std::pair<int, int>> points;
    shadow_cast(
            [&cp, &cx, &cy](int i, int j) {
                return ((unsigned int) cp->getCost(cx + i, cy + j)) > 200;
            },
            [&points, &cx, &cy](int i, int j) {
                points.emplace_back(cx + i, cy + j);
            },
            // TODO : High distance causes stack overflow
            (int) (4.0 / _costmap->getCostmap()->getResolution())
    );
    std::cout << "Obstacle points: " << points.size() << std::endl;


    if (points.size() < 4)
        return {};


    std::vector<size_t> split_points;


    // Find gaps between obstacles i.e split_points
    auto &prev = points.back();
    for (size_t i = 0; i < points.size() - 1; i++) {
        const auto &cur = points[i];
        if ((abs(cur.first - prev.first) + abs(cur.second - prev.second)) > 3) {
            split_points.push_back(i);
        }
        prev = cur;
    }


    if (split_points.empty())
        return {};

    // The obstacle points are arranged circularly around the robot, but vector has a start and end.
    // We make sure that the obstacle points belonging to one obstacle remain together
    // The obstacle which has some points at the end and some at the start is completely shifted to the front
    // Now we ignore the first split_pts[0] points as it is moved to the end
    //points.reserve(points.size() + split_pts.front());
    points.insert(points.end(), points.begin(), points.begin() + split_points.front());


    std::cout << "Section sizes: ";
    split_points.push_back(points.size());
    for (int i = 0; i < split_points.size() - 1; i++) {
        std::cout << split_points[i + 1] - split_points[i] << " ";
    }
    std::cout << std::endl;

    // Transform points to robot frame
    std::vector<double> pts_x, pts_y; // The transformed points.
    pts_x.resize(points.size());
    pts_y.resize(points.size());

    for (int i = split_points[0]; i < points.size(); i++) {
        // Costmap to world
        double xx, yy;
        cp->mapToWorld(points[i].first, points[i].second, xx, yy);

        // World to robot
        const double shift_x = xx - x;
        const double shift_y = yy - y;
        pts_x[i] = shift_x * cos(-yaw) - shift_y * sin(-yaw);
        pts_y[i] = shift_x * sin(-yaw) + shift_y * cos(-yaw);
    }

    // TODO: Publish obstacle points
    _pc_pub->clear_cloud();
    _pc_pub->header->frame_id = _costmap->getBaseFrameID();
    _pc_pub->header->stamp = ros::Time::now();
    auto[xi, yi, zi] = _pc_pub->get_iter(pts_x.size());
    for (size_t i = 0; i < pts_x.size(); i++) {
        *xi = pts_x[i];
        *yi = pts_y[i];
        *zi = 0;
        ++xi, ++yi, ++zi;
    }
    _pc_pub->publish();

    // Fit each obstacle to polynomial
    std::cout << "Fitting" << std::endl;
    for (int i = 0; i < split_points.size() - 1; i++) {
        const auto pts_in_obstacle = split_points[i + 1] - split_points[i];
        if (pts_in_obstacle < 4) continue;

        Eigen::VectorXd poly_coeffs = polyfit(
                Eigen::Map<Eigen::VectorXd>(pts_x.data() + split_points[i], pts_in_obstacle),
                Eigen::Map<Eigen::VectorXd>(pts_y.data(), pts_in_obstacle),
                3);


        polynomials.emplace_back();

        poly_coeffs.resize(3); // Ensure we have 3 values
        polynomials.back() = {poly_coeffs[0], poly_coeffs[1], poly_coeffs[2]};
    }

    std::cout << "Num obstacle polnomial: " << polynomials.size();

    return polynomials;
}

// Gets transformation from global frame to robot frame, i.e robot position.
// @return success ( bool )
/*bool MPC_Local_Planner::get_trans(double &x, double &y, double &yaw) const {
    try {
        auto trans = _tf_buffer->lookupTransform(_costmap->getGlobalFrameID(), _costmap->getBaseFrameID(),
                                                 ros::Time(0)).transform;
        x = trans.translation.x;
        y = trans.translation.y;
        double _roll, _pitch;
        tf2::Matrix3x3(tf2::Quaternion{trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w}).getRPY(
                _roll, _pitch, yaw);
        return true;

    } catch (tf2::TransformException &e) { // Oh no!
        ROS_ERROR("Can't get transform from %s to %s.",
                  _costmap->getGlobalFrameID().c_str(), _costmap->getBaseFrameID().c_str());
        return false;
    }
}*/
bool MPC_Local_Planner::get_trans(double &x, double &y, double &yaw) const {
    geometry_msgs::PoseStamped pose;
    if (!_costmap->getRobotPose(pose))
        return false;

    x = pose.pose.position.x;
    y = pose.pose.position.y;

    double _roll, _pitch;
    tf2::Matrix3x3(tf2::Quaternion{pose.pose.orientation.x, pose.pose.orientation.y,
                                   pose.pose.orientation.z, pose.pose.orientation.w}).getRPY(_roll, _pitch, yaw);

    return true;
}

bool MPC_Local_Planner::isGoalReached() {
    if (!isInitialized()) {
        ROS_ERROR("This planner not initialized.");
        return false;
    }

    return _plan.first.empty();
}

void MPC_Local_Planner::publish_plan(const std::vector<mpc_lib::State> &plan) {
    /*if (!_pub)
        return;*/
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = _costmap->getBaseFrameID(); // Base <=> robot

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = _costmap->getBaseFrameID();

    for (const auto &s : plan) {
        pose.pose.position.x = s.x;
        pose.pose.position.y = s.y;

        pose.pose.orientation.z = cos(s.theta);
        pose.pose.orientation.w = sin(s.theta);

        path.poses.emplace_back(pose);
    }

    _path_pub->publish(path);
}
