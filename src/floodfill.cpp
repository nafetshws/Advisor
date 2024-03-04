#include "../include/floodfill.hpp"

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