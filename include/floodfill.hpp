#ifndef FLOODFILL_HPP
#define FLOODFILL_HPP

#include <cstdint>

/**
 * Represents a cell using only 4 bytes.
 * Example represantation of walls: xxxx1110
 * -> North wall: first bit
 * -> East wall: second bit
 * -> Sout wall: third bit
 * -> West wall: fourth bit
 * In this case the cell is surrounded by walls except for the north side 
*/
static const int SIZE = 16;

struct Cell {
    uint8_t x;
    uint8_t y;
    uint8_t distance;
    uint8_t walls;

    Cell() : x(0), y(0), distance(0), walls(0) {}
    Cell(uint8_t x, uint8_t y) : x(x), y(y), distance(0), walls(0) {}

    bool hasNorthWall();
    bool hasEastWall();
    bool hasSoutWall();
    bool hasWestWall();

    void setNorthWall();
    void setEastWall();
    void setSoutWall();
    void setWestWall();
};

extern Cell maze[SIZE][SIZE];

//Numbering based on quadrants of a Cartesian coordinate system
extern Cell *center1;
extern Cell *center2;
extern Cell *center3;
extern Cell *center4;


uint8_t calculateManhattanDistance(const Cell& cell1, const Cell& cell2);
void initMaze();


#endif