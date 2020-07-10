#ifndef MPC_LOCAL_PLANNER_POLYFIT_H
#define MPC_LOCAL_PLANNER_POLYFIT_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

namespace mpc_local_planner {
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
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
}

#endif //MPC_LOCAL_PLANNER_POLYFIT_H
