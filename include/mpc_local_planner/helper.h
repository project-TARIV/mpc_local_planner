#ifndef MPC_LOCAL_PLANNER_HELPER_H
#define MPC_LOCAL_PLANNER_HELPER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>


inline double signum(double x) {
    return (x > 0) - (x < 0);
}

namespace helper {
    Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);

    class PCPublisher {
        ros::Publisher _pub;
        sensor_msgs::PointCloud2Ptr pc;

    public:
        sensor_msgs::PointCloud2::_header_type *header{};

        explicit PCPublisher(const ros::Publisher &pub);

        void clear_cloud();

        std::array<sensor_msgs::PointCloud2Iterator<float>, 3> get_iter(unsigned int size);

        void publish();
    };
}
#endif //MPC_LOCAL_PLANNER_HELPER_H
