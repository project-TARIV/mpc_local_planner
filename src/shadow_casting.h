#ifndef MPC_LOCAL_PLANNER_SHADOW_CASTING_H
#define MPC_LOCAL_PLANNER_SHADOW_CASTING_H

#include <functional>

//typedef bool IsBlocked(int, int); // convert to using?
//typedef void AddPoint(int, int); // convert to using?

using IsBlocked = std::function<bool(int, int)>;
using AddPoint = std::function<void(int, int)>;

void shadow_cast(IsBlocked is_blocked, AddPoint add_point, double m_start, double m_end,
                 const int i, const int i_max, const int j_max);

#endif //MPC_LOCAL_PLANNER_SHADOW_CASTING_H
