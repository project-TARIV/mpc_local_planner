#include "../include/mpc_local_planner/mpc_local_planner.h"
#include "../include/mpc_local_planner/shadow_casting.h"

// implement MPC_Local_Planner
using namespace mpc_local_planner;

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
    for (int segment_num = 0; segment_num < split_points.size() - 1; segment_num++) {
        const auto pts_in_obstacle = split_points[segment_num + 1] - split_points[segment_num];
        if (pts_in_obstacle < 4) continue;


        // Here, we are going to add some extra points, to ensure the polynomial goes away from the robot
        // This is pretty inefficient, the entire set of points is being copied each time.
        std::vector<double> x_doctored(pts_x.data() + split_points[segment_num],
                                       pts_x.data() + split_points[segment_num] + pts_in_obstacle);
        x_doctored.resize(pts_in_obstacle + 6);
        std::vector<double> y_doctored(pts_y.data() + split_points[segment_num],
                                       pts_y.data() + split_points[segment_num] + pts_in_obstacle);
        y_doctored.resize(pts_in_obstacle + 6);

        auto xit = x_doctored.end();
        auto &x_first = x_doctored.front();
        *(--xit) = x_first;
        *(--xit) = x_first;
        *(--xit) = x_first;

        auto &last_x = x_doctored[x_doctored.size() - 7];
        *(--xit) = last_x;
        *(--xit) = last_x;
        *(--xit) = last_x;

        auto yit = y_doctored.end();
        auto &y_first = y_doctored.front();
        auto inc_first = signum(y_first);
        *(--yit) = y_first + 1 * inc_first;
        *(--yit) = y_first + 2 * inc_first;
        *(--yit) = y_first + 3 * inc_first;

        auto &last_y = y_doctored[y_doctored.size() - 7];
        auto inc_last = signum(last_y);
        *(--yit) = last_y + 3 * inc_last;
        *(--yit) = last_y + 2 * inc_last;
        *(--yit) = last_y + 1 * inc_last;

        std::rotate(x_doctored.begin(),
                    x_doctored.end() - 3, // this will be the new first element
                    x_doctored.end());

        std::rotate(y_doctored.begin(),
                    y_doctored.end() - 3, // this will be the new first element
                    y_doctored.end());

        /*std::cout << "______________________" << std::endl;
        for (size_t index = 0; index < x_doctored.size(); index++) {
            std::cout << x_doctored[index] << "\t" << y_doctored[index] << std::endl;
        }*/

        // Get obstacle poly
        Eigen::VectorXd poly_coeffs = polyfit(
                Eigen::Map<Eigen::VectorXd>(x_doctored.data(), x_doctored.size()),
                Eigen::Map<Eigen::VectorXd>(y_doctored.data(), y_doctored.size()),
                3);

        polynomials.emplace_back();

        // TODO: This seems to be changing the values
        // poly_coeffs.resize(3); // Ensure we have 3 values

        polynomials.back() = {poly_coeffs[0], poly_coeffs[1], poly_coeffs[2], poly_coeffs[3]};
    }

    std::cout << "Num obstacle polynomial: " << polynomials.size();

    return polynomials;
}
