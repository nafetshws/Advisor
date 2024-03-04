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