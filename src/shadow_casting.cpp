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

void shadow_cast_octent(IsBlocked is_blocked, AddPoint add_point,
                        double m_start, double m_end,
                        const int i, const int i_max,
                        const int j_max, const double del) {
    // assert( 0 <= m_end < m_start <= 1);
    if (i >= i_max) {
        return;
    }
    if (m_start < m_end + del) {
        return;
    }

    bool in_block = true;

    const int j_start = std::min((int) std::floor(m_start * (i + 0.5) + 0.5 - del), j_max - 1);
    const int j_end = (int) std::ceil(m_end * (i - 0.5) - 0.5 + del);

    if (j_start < j_end) {
        return;
    }

    int j = j_start + 1; // + 1 compensates for
    while (--j >= j_end) {
        if (in_block) {
            if (is_blocked(i, j)) {
                add_point(i, j);
                m_start = (j - 0.5) / (i + 0.5);
            } else {
                in_block = false;
            }
        } else {
            if (is_blocked(i, j)) {
                in_block = true;
                const double _m_end = (j + 0.5) / (i - 0.5); // m_end for recursive section
                shadow_cast_octent(is_blocked, add_point, m_start, _m_end, i + 1, i_max, j_max, 0);

                add_point(i, j);
                m_start = (j - 0.5) / (i + 0.5);
            }
        }
    }
    if (!in_block) {
        shadow_cast_octent(is_blocked, add_point, m_start, m_end, i + 1, i_max, j_max, 0);
    }
}

void shadow_cast_quad(const std::pair<int, int> dir,
                      IsBlocked is_blocked, AddPoint add_point,
                      const int max_index, const double del) {
    shadow_cast_octent(
            [&is_blocked, &dir](int i, int j) {
                return is_blocked(dir.first * i, dir.second * j);
            },
            [&add_point, &dir](int i, int j) {
                add_point(dir.first * i, dir.second * j);
            },
            1, 0, 1,
            max_index, max_index, del);
    shadow_cast_octent(
            [&is_blocked, &dir](int i, int j) {
                return is_blocked(dir.first * j, dir.second * i);
            },
            [&add_point, &dir](int i, int j) {
                add_point(dir.first * j, dir.second * i);
            },
            1, 0, 1,
            max_index, max_index, del);
}

void shadow_cast(IsBlocked is_blocked, AddPoint add_point,
                 const int max_index, const double del) {
    shadow_cast_quad({1, 1}, is_blocked, add_point, max_index, del);
    shadow_cast_quad({1, -1}, is_blocked, add_point, max_index, del);
    shadow_cast_quad({-1, -1}, is_blocked, add_point, max_index, del);
    shadow_cast_quad({-1, 1}, is_blocked, add_point, max_index, del);
}
