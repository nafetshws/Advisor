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
struct Cell {
    uint8_t x;
    uint8_t y;
    uint8_t distance;
    uint8_t walls;

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

#endif