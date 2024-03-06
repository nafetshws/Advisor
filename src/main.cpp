#include <iostream>
#include <string>
#include "../include/misc.hpp"
#include "../include/mms.hpp"
#include "../include/floodfill.hpp"

int main() {
    Maze::initMaze();

    for(int y = 0; y < SIZE; y++) {
        for(int x = 0; x < SIZE; x++) {
            MMS::setText(x, y, std::to_string(Maze::get(x, y)->distance));
        }
    }
    floodfill(*Maze::startCell);
}