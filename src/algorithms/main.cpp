#include <iostream>
#include <string>
#include "../../include/mms.hpp"
#include "../../include/floodfill.hpp"

int main() {
    //Floodfill
    Maze::initMaze();

    for(int y = 0; y < SIZE; y++) {
        for(int x = 0; x < SIZE; x++) {
            MMS::setText(x, y, std::to_string(Maze::get(x, y)->distance));
        }
    }
    floodfill(*Maze::startCell);
    optimisePath();

    // Reverse Floodfill
    Maze::initMazeReverse();
    for (int y = 0; y < SIZE; y++)
    {
        for (int x = 0; x < SIZE; x++)
        {
            MMS::setText(x, y, std::to_string(Maze::get(x, y)->distance));
        }
    }

    floodfill(*Maze::endCell, direction_last);
    optimisePath();
}