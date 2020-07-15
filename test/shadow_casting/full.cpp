//
// Created by suraj on 7/15/20.
//

#include <vector>
#include <iostream>

#include "../../src/shadow_casting.h"

const size_t size = 20;


void do_quad(bool board[][size], const size_t cx, const size_t cy,
             std::vector<std::pair<int, int>> &points, int dir_1, int dir_2) {
    shadow_cast(
            [&board, cx, cy, dir_1, dir_2](int i, int j) {
                return board[cx + dir_1 * i][cy + dir_2 * j];
            },
            [&points, cx, cy, dir_1, dir_2](int i, int j) {
                points.emplace_back(cx + dir_1 * i, cy + dir_2 * j);
            },
            1, 0, 1, size / 2, size / 2);
    shadow_cast(
            [&board, cx, cy, dir_1, dir_2](int i, int j) {
                return board[cx + dir_1 * j][cy + dir_2 * i];
            },
            [&points, cx, cy, dir_1, dir_2](int i, int j) {
                points.emplace_back(cx + dir_1 * j, cy + dir_2 * i);
            },
            1, 0, 1, size / 2, size / 2);
}

int main() {
    // python: print(',\n'.join([ '{' + ', '.join('0' for j in range(10)) +'}' for i in range(10)]))

    // Auto generate board defintion using:
    // python -c "n = 20; print('const size_t size = '+str(n)+';\nbool board[size][size] = {\n    ' + '\n    '.join([ '{' + ', '.join('0' for j in range(n)) +' },' for i in range(n)]) + '\n};')"
    /*const size_t size = 10;
    bool board[size][size] = {
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 1, 1, 1, 0, 1},
            {0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
            {0, 0, 0, 1, 1, 1, 1, 1, 1, 0},
            {0, 0, 0, 1, 1, 1, 1, 1, 0, 0},
            {0, 0, 1, 1, 1, 1, 1, 0, 0, 0},
            {0, 0, 1, 1, 1, 1, 1, 0, 0, 0},
            {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 0, 0, 0, 0}
    };*/

    bool board[size][size] = {
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
            {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0},
            {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0},
            {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
    };

    const size_t cx = size / 2;
    const size_t cy = size / 2;

    std::vector<std::pair<int, int>> points;

    do_quad(board, cx, cy, points, 1, 1);
    do_quad(board, cx, cy, points, 1, -1);
    do_quad(board, cx, cy, points, -1, -1);
    do_quad(board, cx, cy, points, -1, 1);

    std::cout << points.size() << " outline points." << std::endl;
    std::cout << "'@' is the robot." << std::endl;
    std::cout << "'.' is empty space." << std::endl;
    std::cout << "'=' is obstacle in the map." << std::endl;
    std::cout << "'#' is an outline point." << std::endl;

    char output[size][size];
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            output[i][j] = board[i][j] ? '=' : '.';
        }
    }
    output[cx][cy] = '@';

    const bool letter_print = false;
    int index = 0;
    for (const auto &[i, j] : points) {
        output[i][j] = !letter_print ? '#' : 'A' + index;
        index++; // index %= 26;
    }

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            std::cout << output[i][j];
        }
        std::cout << std::endl;
    }
}