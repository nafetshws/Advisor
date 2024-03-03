#include <iostream>
#include "../include/misc.hpp"
#include "../include/mms.hpp"

int main() {
    LOG("Running...");
    MMS::setColor(0, 0, 'G');
    MMS::setText(0, 0, "abc");
    while (true) {
        if (!MMS::wallLeft()) {
            MMS::turnLeft();
        }
        while (MMS::wallFront()) {
            MMS::turnRight();
        }
        MMS::moveForward();
    }
}