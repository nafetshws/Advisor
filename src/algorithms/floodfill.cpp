#include <vector>
#include <algorithm>
#include <cmath>
#include <stack>
#include <iostream>
#include "../../include/floodfill.hpp"
#include "../../include/mms.hpp"

// static variables
Cell Maze::maze[SIZE][SIZE];
Cell *Maze::center1;
Cell *Maze::center2;
Cell *Maze::center3;
Cell *Maze::center4;
Cell *Maze::startCell;
Cell *Maze::endCell;
std::vector<Cell*> Maze::floodfillPath;
std::vector<Cell*> Maze::floodfillReversePath;
bool Maze::reverseMode;

int direction_last;

void Maze::initMaze()
{
    // init x and y values
    for (int y = 0; y < SIZE; y++)
    {
        for (int x = 0; x < SIZE; x++)
        {
            Maze::get(x, y)->x = x;
            Maze::get(x, y)->y = y;
            Maze::get(x, y)->discovered = false;

            //set walls that surround the maze
            if(y == 0) {
                Maze::get(x, y)->setSoutWall();
                MMS::setWall(x, y, 's');
            } 
            if(y == SIZE-1) {
                Maze::get(x, y)->setNorthWall(); 
                MMS::setWall(x, y, 'n');
            }
            if(x == 0) {
                Maze::get(x, y)->setWestWall(); 
                MMS::setWall(x, y, 'w');
            } 
            if(x == SIZE-1) {
                Maze::get(x, y)->setEastWall(); 
                MMS::setWall(x, y, 'e');
            }
        }
    }

    //init global center cells
    Maze::center1 = Maze::get(8, 8);
    Maze::center2 = Maze::get(7, 8);
    Maze::center3 = Maze::get(7, 7);
    Maze::center4 = Maze::get(8, 7);

    //calculate Manhattan distance of each cell
    for(int y = 0; y < SIZE; y++) {
        for(int x = 0; x < SIZE; x++) {
            std::vector<uint8_t> md = {
                calculateManhattanDistance(*Maze::get(x, y), *center1),
                calculateManhattanDistance(*Maze::get(x, y), *center2),
                calculateManhattanDistance(*Maze::get(x, y), *center3),
                calculateManhattanDistance(*Maze::get(x, y), *center4),
            };
            //choose lowest md
            Maze::get(x, y)->distance = *(std::min_element(md.begin(), md.end()));
        }
    }
    //set start cell
    Maze::startCell = Maze::get(0, 0);
}

void Maze::initMazeReverse()
{
    Maze::reverseMode = true;
    Maze::center1 = Maze::get(0, 0);

    // calculate Manhattan distance of each cell
    for (int y = 0; y < SIZE; y++)
    {
        for (int x = 0; x < SIZE; x++)
        {
            Maze::get(x, y)->distance = calculateManhattanDistance(*Maze::get(x, y), *center1);
        }
    }
    // set start cell
    Maze::startCell = Maze::endCell;
}

Cell *Maze::get(uint8_t x, uint8_t y)
{
    return &Maze::maze[y][x];
}

void floodfill(Cell &c, int direction)
{
    floodfillHelper(c, direction);
}

// Modified flood fill algorithm
void floodfillHelper(Cell &c, int direction)
{
    // add visited cell to path of robot
    if (Maze::reverseMode) {
        Maze::floodfillReversePath.push_back(&c);
    } else {
        Maze::floodfillPath.push_back(&c);
    }

    if (c.distance == 0)
    {
        direction_last = direction;
        Maze::endCell = &c;
        std::cerr << "direction:" << direction << " direction_last:" << direction_last << std::endl;

        return;
    }

    //update walls
    updateWalls(c, direction);
    c.discovered = true;

    std::stack<Cell*> stack;
    stack.push(&c);

    Cell *nextCell;

    while(!stack.empty()) {
        //pop elememt from stack
        Cell *current = stack.top();
        stack.pop();

        std::vector<Cell*> neighbors;

        if(current->y < SIZE - 1) neighbors.push_back(Maze::get(current->x, current->y + 1)); // add north neighbor
        if(current->x < SIZE - 1) neighbors.push_back(Maze::get(current->x + 1, current->y)); // add east neighbor
        if(current->y > 0) neighbors.push_back(Maze::get(current->x, current->y - 1));        // add south neighbor
        if(current->x > 0) neighbors.push_back(Maze::get(current->x - 1, current->y));        // add west neighbor

        //find minimum Manhattan distance of open neighbors
        int minMD = MAX_PATH_LENGTH;
        for(Cell *neighbor : neighbors) {
            //std::cerr << "Neighbor cell: (" << unsigned(neighbor->x) << ", " << unsigned(neighbor->y) << ")" << std::endl;
            if(!isOpenNeighbor(*current, *neighbor)) continue;

            if(neighbor->distance < minMD) {
                minMD = neighbor->distance;
                //update the next cell only if the neighbors of the current cell, which the robot is in, are searched
                if(current->x == c.x && current->y == c.y) {
                    nextCell = neighbor;
                }
            }
        }

        if(current->distance - 1 != minMD) {
            //inc distance
            current->distance = minMD + 1;

            //update value of square
            MMS::setText(current->x, current->y, std::to_string(current->distance)); 

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
    floodfillHelper(*nextCell, direction);
}

void updateWalls(Cell& cell, int direction) {
    if(MMS::wallFront()) {
        Maze::get(cell.x, cell.y)->setWall(DIRECTIONS[direction]);
        MMS::setWall(cell.x, cell.y, DIRECTIONS[direction]);
    }

    if(MMS::wallRight()) {
        Maze::get(cell.x, cell.y)->setWall(DIRECTIONS[(direction + 1) % 4]);
        MMS::setWall(cell.x, cell.y, DIRECTIONS[(direction + 1) % 4]);
    }

    if(MMS::wallLeft()) {
        int newDirection = mod(direction - 1, 4);
        Maze::get(cell.x, cell.y)->setWall(DIRECTIONS[newDirection]);
        MMS::setWall(cell.x, cell.y, DIRECTIONS[newDirection]);
    }
}

void Cell::setWall(char direction) {
    switch(direction) {
        case 'n':
            Maze::get(this->x, this->y)->setNorthWall();
            break;
        case 'e':
            Maze::get(this->x, this->y)->setEastWall();
            break;
        case 's':
            Maze::get(this->x, this->y)->setSoutWall();
            break;
        case 'w':
            Maze::get(this->x, this->y)->setWestWall();
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

//Necessary because % isn't a real modulo operator but a remainder operator
int mod(int a, int b) {
    int res = a % b;
    if(res < 0) {
        res += b;
    }
    return res;
}