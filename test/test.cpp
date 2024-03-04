#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../include/doctest.h"
#include "../include/misc.hpp"
#include "../include/floodfill.hpp"

TEST_CASE("testing factorial") {
	CHECK(factorial(0) == 1);
	CHECK(factorial(1) == 1);
	CHECK(factorial(2) == 2);
	CHECK(factorial(3) == 6);
	CHECK(factorial(4) == 24);
}

TEST_CASE("testing wall representation of a cell") {
	Cell cell(1, 2);
	//no walls assigned yet
	CHECK(cell.hasNorthWall() == false);
	CHECK(cell.hasEastWall() == false);
	CHECK(cell.hasSoutWall() == false);
	CHECK(cell.hasWestWall() == false);

	cell.setNorthWall();
	cell.setWestWall();

	CHECK(cell.hasNorthWall() == true);
	CHECK(cell.hasEastWall() == false);
	CHECK(cell.hasSoutWall() == false);
	CHECK(cell.hasWestWall() == true);

	cell.setSoutWall();
	cell.setEastWall();

	CHECK(cell.hasNorthWall() == true);
	CHECK(cell.hasEastWall() == true);
	CHECK(cell.hasSoutWall() == true);
	CHECK(cell.hasWestWall() == true);
}

TEST_CASE("testing maze initialization") {
	initMaze();

	//check distances - center
	CHECK(center1->distance == 0);
	CHECK(center2->distance == 0);
	CHECK(center3->distance == 0);
	CHECK(center4->distance == 0);

	//check distances - corner
	CHECK(maze[0][0].distance == 14);
	CHECK(maze[0][SIZE-1].distance == 14);
	CHECK(maze[SIZE-1][0].distance == 14);
	CHECK(maze[SIZE-1][SIZE-1].distance == 14);

	//check distances - random squares
	CHECK(maze[5][6].distance == 3); //(x, y) = (6, 5)
	CHECK(maze[13][3].distance == 9); 
	CHECK(maze[13][13].distance == 10);
	CHECK(maze[1][11].distance == 9); 
}