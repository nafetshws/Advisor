#include <vector>
#include <algorithm>
#include <cmath>
#include "../include/floodfill.hpp"


//necessary for linker - global variables
Cell maze[SIZE][SIZE];
Cell *center1;
Cell *center2;
Cell *center3;
Cell *center4;

void initMaze() {
    //init x and y values
    for(int y = 0; y < SIZE; y++) {
        for(int x = 0; x < SIZE; x++) {
            maze[y][x].x = x;
            maze[y][x].y = y;
        }
    }

    //init global center cells
    center1 = &maze[8][8];
    center2 = &maze[8][7];
    center3 = &maze[7][7];
    center4 = &maze[7][8];

    //calculate Manhattan distance of each cell
    for(int y = 0; y < SIZE; y++) {
        for(int x = 0; x < SIZE; x++) {
            std::vector<uint8_t> md = {
                calculateManhattanDistance(maze[y][x], *center1),
                calculateManhattanDistance(maze[y][x], *center2),
                calculateManhattanDistance(maze[y][x], *center3),
                calculateManhattanDistance(maze[y][x], *center4),
            };
            //choose lowest md
            maze[y][x].distance = *(std::min_element(md.begin(), md.end()));
        }
    }
}

uint8_t calculateManhattanDistance(const Cell& cell1, const Cell& cell2) {
    return std::abs(cell1.x - cell2.x) + std::abs(cell1.y - cell2.y);
}

bool Cell::hasNorthWall() {
    return (1 & this->walls) != 0;
}

bool Cell::hasEastWall() {
    return (2 & this->walls) != 0;
}

bool Cell::hasSoutWall() {
    return (4 & this->walls) != 0;
}

bool Cell::hasWestWall() {
    return (8 & this->walls) != 0;
}

void Cell::setNorthWall() {
    this->walls |= 1;
}

void Cell::setEastWall() {
    this->walls |= 2;
}

void Cell::setSoutWall() {

    this->walls |= 4;
}

void Cell::setWestWall() {
    this->walls |= 8;
}