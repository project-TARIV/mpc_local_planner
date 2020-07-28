#include "../include/mpc_local_planner/helper.h"


Eigen::VectorXd helper::polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    Eigen::VectorXd result = A.householderQr().solve(yvals);

    return result;
}

namespace helper {
    PCPublisher::PCPublisher(const ros::Publisher &pub) : _pub(pub), header(nullptr) {

    }

    using pc_iter = sensor_msgs::PointCloud2Iterator<float>;

    std::array<pc_iter, 3> PCPublisher::get_iter(unsigned int size) {
        pc->width = size;

        sensor_msgs::PointCloud2Modifier(*pc).setPointCloud2FieldsByString(1, "xyz");

        return {
                pc_iter(*pc, "x"),
                pc_iter(*pc, "y"),
                pc_iter(*pc, "z"),
        };
    }

    void PCPublisher::clear_cloud() {
        pc = boost::make_shared<sensor_msgs::PointCloud2>();
        header = &pc->header;

        pc->height = 1;
        pc->is_bigendian = false;
        pc->is_dense = false;
    }

    void PCPublisher::publish() {
        _pub.publish(pc);
    }
}