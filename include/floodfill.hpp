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
static const int MAX_PATH_LENGTH = SIZE * SIZE;
static const char DIRECTIONS[4] = {'n', 'e', 's', 'w'};

struct Cell
{
    uint8_t x;
    uint8_t y;
    uint8_t distance;
    uint8_t walls;

    Cell() : x(0), y(0), distance(0), walls(0) {}
    Cell(uint8_t x, uint8_t y) : x(x), y(y), distance(0), walls(0) {}

    bool discovered;

    bool hasNorthWall();
    bool hasEastWall();
    bool hasSoutWall();
    bool hasWestWall();

    void setNorthWall();
    void setEastWall();
    void setSoutWall();
    void setWestWall();

    void setWall(char direction);
};

struct Maze
{
public:
    static Cell maze[SIZE][SIZE];

    static void initMaze();
    static void initMazeReverse();
    static Cell *get(uint8_t x, uint8_t y);

    // Numbering based on quadrants of a Cartesian coordinate system
    static Cell *center1;
    static Cell *center2;
    static Cell *center3;
    static Cell *center4;

    static Cell *startCell;
    static Cell *endCell; // cell where the previous run has ended
};

uint8_t calculateManhattanDistance(const Cell &cell1, const Cell &cell2);
void floodfill(Cell &c, int direction = 0);
void floodfillHelper(Cell &c, int direction);
bool isOpenNeighbor(Cell &c, Cell &neighbor);
void updateWalls(Cell &cell, int direction);
int mod(int a, int b);

extern int direction_last;

#endif