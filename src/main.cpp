#include <iostream>
#include <string>
#include "../include/misc.hpp"
#include "../include/mms.hpp"
#include "../include/floodfill.hpp"

int main() {
    initMaze();
    for(int y = 0; y < SIZE; y++) {
        for(int x = 0; x < SIZE; x++) {
            MMS::setText(x, y, std::to_string(maze[y][x].distance));
        }
    }
}