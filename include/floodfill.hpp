#ifndef FLOODFILL_HPP
#define FLOODFILL_HPP

#include <cstdint>
#include <vector>
#include <queue>
#include <set>
#include <map>
#ifndef DONT_INCLUDE_LIBS 
#include "./robot.hpp"
#endif

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

extern int direction_last;

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

struct Maze {
    public:
        static Cell maze[SIZE][SIZE];
        static std::vector<Cell*> floodfillPath;
        static std::vector<Cell*> floodfillReversePath;

        static bool reverseMode;

        // Numbering based on quadrants of a Cartesian coordinate system
        static Cell *center1;
        static Cell *center2;
        static Cell *center3;
        static Cell *center4;

        static Cell *startCell;
        static Cell *endCell; // cell where the previous run has ended

        // Robot
        #ifndef DONT_INCLUDE_LIBS
        static Robot *robot;
        #endif
        static bool isRobotAttached;

        static void initMaze();
        static void initMazeReverse();
        static Cell *get(uint8_t x, uint8_t y);
        #ifndef DONT_INCLUDE_LIBS
        static void attachRobot(Robot *r);
        static void dettachRobot();
        #endif
        static bool getIsRobotAttached();
};

struct Graph 
{
    std::set<Cell*> vertices;
    std::set<std::set<Cell*>> edges;
    std::map<Cell*, std::set<Cell*>> graph;

    std::map<Cell*, Cell*> par;
    std::map<Cell*, int> dist;

    void addVertex(Cell* c);
    void addEdge(Cell* a, Cell* b);

    void bfsInit();
    void print(Cell* S, Cell* D);
    void bfs(Cell* S);
};

extern Graph g;

uint8_t calculateManhattanDistance(const Cell &cell1, const Cell &cell2);
void floodfill(Cell &c, int direction = 0);
void floodfillHelper(Cell &c, int direction);
bool isOpenNeighbor(Cell &c, Cell &neighbor);
void updateWalls(Cell &cell, int direction);
int mod(int a, int b);
void optimisePath();

extern int direction_last;

// Movement functions of robot
void turnRight();
void turnLeft();
void moveForward(int distance = 1);

bool wallFront();
bool wallRight();
bool wallLeft();

#endif