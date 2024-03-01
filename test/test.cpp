#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../include/doctest.h"
#include "../include/misc.hpp"

TEST_CASE("testing factorial") {
	CHECK(factorial(0) == 1);
	CHECK(factorial(1) == 1);
	CHECK(factorial(2) == 2);
	CHECK(factorial(3) == 6);
	CHECK(factorial(4) == 24);
}

TEST_CASE("testing the sumUpTo function") {
    CHECK(sumUpTo(1) == 1);
    CHECK(sumUpTo(2) == 3);
    CHECK(sumUpTo(3) == 6);
    CHECK(sumUpTo(-10) == 1);
}