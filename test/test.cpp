#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../include/doctest.h"
#include "../include/floodfill.hpp"

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
	Maze::initMaze();

	//check distances - center
	CHECK(Maze::center1->distance == 0);
	CHECK(Maze::center2->distance == 0);
	CHECK(Maze::center3->distance == 0);
	CHECK(Maze::center4->distance == 0);

	//check distances - corner
	CHECK(Maze::get(0, 0)->distance == 14);
	CHECK(Maze::get(SIZE-1, 0)->distance == 14);
	CHECK(Maze::get(0, SIZE-1)->distance == 14);
	CHECK(Maze::get(SIZE-1, SIZE-1)->distance == 14);

	//check distances - random squares
	CHECK(Maze::get(6, 5)->distance == 3); //(x, y) = (6, 5)
	CHECK(Maze::get(3, 13)->distance == 9); 
	CHECK(Maze::get(13, 13)->distance == 10);
	CHECK(Maze::get(11, 1)->distance == 9); 
}