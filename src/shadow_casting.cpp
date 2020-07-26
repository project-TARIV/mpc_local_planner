#include <cmath>
#include <algorithm>
#include "shadow_casting.h"

/*
 * Shadow casts  on the lower triangle with base i:0...i_max between the slopes m_end and m_start, where m_end < m_start.
 * Obstacle outlines are invoked with add_point.
 * Thus points are added from diagonal in.
 *
 * Based on 'http://www.roguebasin.com/index.php?title=FOV_using_recursive_shadowcasting'
 */

void shadow_cast_octet(const IsBlocked &is_blocked, const AddPoint &add_point,
                       double m_start, double m_end,
                       const int i, const int max_dist,
                       const double del) {

    if (i >= max_dist) return;

    if (m_start < m_end + del) return;

    // Was the prev i,j blocked
    bool in_block = true;

    // Find j indices and bind them within reasonable range
    const int j_start = std::min((int) std::floor(m_start * (i + 0.5) + 0.5 - del), max_dist - i);
    const int j_end = std::max((int) std::ceil(m_end * (i - 0.5) - 0.5 + del), 0);

    if (j_start < j_end) return;

    int j = j_start + 1; // + 1 compensates for
    // TODO: I dont know why +1 ^^^^^^^
    while (--j >= j_end) {
        if (in_block) {
            if (is_blocked(i, j)) {
                // Is a boundary of an obstacle
                if (i >= j) // HACK: I dont know why we are getting some incorrect points
                    add_point(i, j);

                m_start = (j - 0.5) / (i + 0.5);
            } else {
                in_block = false;
            }
        } else {
            if (is_blocked(i, j)) {
                in_block = true;
                // An open area just ended, so need to start shadow casting for that region
                //const double _m_end = (j + 0.5) / (i - 0.5); // m_end for recursive section
                const double _m_end = (j) / (i - 0.5); // m_end for recursive section
                shadow_cast_octet(is_blocked, add_point, m_start, _m_end, i + 1, max_dist, del);

                if (i >= j) // HACK: I dont know why we are getting some incorrect points
                    add_point(i, j);
                m_start = (j - 0.5) / (i + 0.5);
            }
        }
    }

    if (!in_block) { // Check if an open area was left hanging
        // Shadow cast for it
        shadow_cast_octet(is_blocked, add_point, m_start, m_end, i + 1, max_dist, del);
    }
}

void shadow_cast_quad(const std::pair<int, int> dir,
                      const IsBlocked &is_blocked, const AddPoint &add_point,
                      const int max_dist, const double del) {

    const auto parity = dir.first * dir.second;

    if (parity == 1) {
        // Normal Octet
        std::vector<std::pair<int, int>> pts;
        shadow_cast_octet(
                [&is_blocked, &dir](int i, int j) {
                    return is_blocked(dir.first * i, dir.second * j);
                },
                [&pts, &dir](int i, int j) {
                    if (j != 0 && i != j)
                        pts.emplace_back(dir.first * i, dir.second * j);
                },
                1, 0, 1,
                max_dist, del);

        for (auto i = pts.rbegin(); i != pts.rend(); ++i) {
            add_point(i->first, i->second);
        }


        // Mirrored octet
        // i.e just swap the i and j co-ordinates
        shadow_cast_octet(
                [&is_blocked, &dir](int i, int j) {
                    return is_blocked(dir.first * j, dir.second * i);
                },
                [&add_point, &dir](int i, int j) {
                    add_point(dir.first * j, dir.second * i);
                },
                1, 0, 1,
                max_dist, del);

    } else if (parity == -1) {
        // Flip order to go counter clockwise
        std::vector<std::pair<int, int>> pts;
        // Mirrored octet
        shadow_cast_octet(
                [&is_blocked, &dir](int i, int j) {
                    return is_blocked(dir.first * j, dir.second * i);
                },
                [&pts, &dir](int i, int j) {
                    if (j != 0 && i != j)
                        pts.emplace_back(dir.first * j, dir.second * i);
                },
                1, 0, 1,
                max_dist, del);

        // Add the points in reverse order, i.e ccw around the robot
        for (auto i = pts.rbegin(); i != pts.rend(); ++i) {
            add_point(i->first, i->second);
        }


        // Normal Octet
        shadow_cast_octet(
                [&is_blocked, &dir](int i, int j) {
                    return is_blocked(dir.first * i, dir.second * j);
                },
                [&add_point, &dir](int i, int j) {
                    add_point(dir.first * i, dir.second * j);
                },
                1, 0, 1,
                max_dist, del);

    } else {
        // ERROR!
    }
}

void shadow_cast(const IsBlocked &is_blocked, const AddPoint &add_point,
                 const int max_index, const double del) {
    // Rotate through quadrants counter-clockwise
    shadow_cast_quad({1, 1}, is_blocked, add_point, max_index, del);
    shadow_cast_quad({-1, 1}, is_blocked, add_point, max_index, del);
    shadow_cast_quad({-1, -1}, is_blocked, add_point, max_index, del);
    shadow_cast_quad({1, -1}, is_blocked, add_point, max_index, del);
}
