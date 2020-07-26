#ifndef MPC_LOCAL_PLANNER_SHADOW_CASTING_H
#define MPC_LOCAL_PLANNER_SHADOW_CASTING_H

#include <functional>

using IsBlocked = std::function<bool(int, int)>;
using AddPoint = std::function<void(int, int)>;

void shadow_cast(const IsBlocked &is_blocked, const AddPoint &add_point, int max_index, double del = 1e-5);

#endif //MPC_LOCAL_PLANNER_SHADOW_CASTING_H
