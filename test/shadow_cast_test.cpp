//
// Created by suraj on 7/15/20.
//

#include <vector>
#include <iostream>
#include <algorithm>

#include "../src/shadow_casting.h"

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

    const size_t size = 20;
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

    std::vector<std::pair<int, int>> points;

    // Lower triangle
    shadow_cast(
            [&board](int i, int j) {
                return board[i][j];
            },
            [&points](int i, int j) {
                points.emplace_back(i, j);
            },
            1, 0, 1, size, size);

    // Upper triangle, mirror of lower triangle
    shadow_cast(
            [&board](int i, int j) {
                return board[j][i];
            },
            [&points](int i, int j) {
                points.emplace_back(j, i);
            },
            1, 0, 1, size, size);


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
    output[0][0] = '@';

    const bool int_print = true;
    int index = 0;
    for (const auto &[i, j] : points) {
        output[i][j] = !int_print ? '#' : 'A' + index;
        index++; // index %= 26;
    }

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            std::cout << output[i][j];
        }
        std::cout << std::endl;
    }
}