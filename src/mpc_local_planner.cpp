#include "../include/mpc_local_planner/mpc_local_planner.h"
//#include "mpc_local_planner/mpc_local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MPC_Local_Planner, nav_core::BaseLocalPlanner)


// implement MPC_Local_Planner
using namespace mpc_local_planner;