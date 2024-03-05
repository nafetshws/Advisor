#include <vector>
#include <algorithm>
#include <cmath>
#include <stack>
#include <iostream>
#include "../include/floodfill.hpp"
#include "../include/mms.hpp"

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

            //set walls that surround the maze
            if(y == 0) {
                maze[y][x].setSoutWall(); 
                MMS::setWall(x, y, 's');
            } 
            if(y == SIZE-1) {
                maze[y][x].setNorthWall();  
                MMS::setWall(x, y, 'n');
            }
            if(x == 0) {
                maze[y][x].setWestWall();
                MMS::setWall(x, y, 'w');
            } 
            if(x == SIZE-1) {
                maze[y][x].setEastWall();
                MMS::setWall(x, y, 'e');
            }
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

void floodfill(Cell& c) {
    floodfillHelper(c, 0);
}

// Modified flood fill algorithm
void floodfillHelper(Cell& c, int direction) {
    if(c.distance == 0) return;

    //update walls
    //LOG("Direction: " + std::string(1, DIRECTIONS[direction]));
    updateWalls(c, direction);

    std::stack<Cell*> stack;
    stack.push(&c);

    Cell *nextCell;

    while(!stack.empty()) {
        //pop elememt from stack
        Cell current = *(stack.top());
        stack.pop();

        std::vector<Cell*> neighbors;
        if(current.y < SIZE - 1) neighbors.push_back(&maze[current.y+1][current.x]); // add north neighbor
        if(current.x < SIZE - 1) neighbors.push_back(&maze[current.y][current.x+1]); // add east neighbor
        if(current.y > 0) neighbors.push_back(&maze[current.y-1][current.x]);        // add south neighbor
        if(current.x > 0) neighbors.push_back(&maze[current.y][current.x-1]);        // add west neighbor

        //find minimum Manhattan distance of open neighbors
        int minMD = MAX_PATH_LENGTH;
        for(Cell *neighbor: neighbors) {
            if(!isOpenNeighbor(current, *neighbor)) continue;

            if(neighbor->distance < minMD) {
                minMD = neighbor->distance;
                nextCell = neighbor;
            }
        }

        if(current.distance - 1 != minMD) {
            //inc distance
            current.distance = minMD + 1;

            //update value of square
            MMS::setText(current.x, current.y, std::to_string(current.distance)); 

            //push neighbors onto the stack
            for(Cell *neighbor : neighbors) {
                //don't add center cells on the stack
                if(neighbor->distance == 0) continue;

                stack.push(neighbor);
            }
        }
    }

    //update direction for nextCell
    int priorDirection = direction;

    if(c.y + 1 == nextCell->y) direction = 0; //north
    else if(c.x + 1 == nextCell->x) direction = 1; //east
    else if(c.y - 1 == nextCell->y) direction = 2; //south
    else direction = 3; //west

    //direction hasn't changed
    if(priorDirection == direction) MMS::moveForward();
    //180 degree turn
    else if(std::abs(priorDirection - direction) == 2) {
        MMS::turnRight(); 
        MMS::turnRight();
        MMS::moveForward();
    } else if((priorDirection + 1) % 4 == direction) {
        MMS::turnRight();
        MMS::moveForward();
    } 
    else {
        MMS::turnLeft();
        MMS::moveForward();
    } 

    //next cell location
    floodfillHelper(*nextCell, direction);
}

void updateWalls(Cell& cell, int direction) {
    std::cerr << "Updating cell: (" << cell.x << cell.y << ")" << std::endl;
    LOG(cell.x + 0);

    if(MMS::wallFront()) {
        LOG("Wall in front");
        maze[cell.y][cell.x].setWall(DIRECTIONS[direction]);
        MMS::setWall(cell.x, cell.y, DIRECTIONS[direction]);
    }

    if(MMS::wallRight()) {
        LOG("Wall right");
        maze[cell.y][cell.x].setWall(DIRECTIONS[(direction + 1) % 4]);
        MMS::setWall(cell.x, cell.y, DIRECTIONS[(direction + 1) % 4]);
    }

    if(MMS::wallLeft()) {
        LOG("Wall left");
        maze[cell.y][cell.x].setWall(DIRECTIONS[(direction - 1) % 4]);
        MMS::setWall(cell.x, cell.y, DIRECTIONS[(direction - 1) % 4]);
    }
}

void Cell::setWall(char direction) {
    switch(direction) {
        case 'n':
            maze[this->y][this->x].setNorthWall();
            break;
        case 'e':
            maze[this->y][this->x].setEastWall();
            break;
        case 's':
            maze[this->y][this->x].setSoutWall();
            break;
        case 'w':
            maze[this->y][this->x].setWestWall();
            break;
        default:
            break;
    }
}

bool isOpenNeighbor(Cell& c, Cell& neighbor) {
    if(c.y + 1 == neighbor.y && !c.hasNorthWall()) return true;
    if(c.x + 1 == neighbor.x && !c.hasEastWall()) return true;
    if(c.y - 1 == neighbor.y && !c.hasSoutWall()) return true;
    if(c.x - 1 == neighbor.x && !c.hasWestWall()) return true;

    return false;
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