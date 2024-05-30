// move along the saved path

#include "../../include/floodfill.hpp"
#include "../../include/manoever.hpp"
#include "../../include/mms.hpp"

void run()
{
    // int direction = 0;

    // for (auto it = path.begin(); it != path.end() - 1; it++)
    // {
    //     int priorDirection = direction;

    //     if (it->y + 1 == (it + 1)->y)
    //         direction = 0; // north
    //     else if (it->x + 1 == (it + 1)->x)
    //         direction = 1; // east
    //     else if (it->y - 1 == (it + 1)->y)
    //         direction = 2; // south
    //     else
    //         direction = 3; // west

    //     // direction hasn't changed
    //     if (priorDirection == direction)
    //         MMS::moveForward();
    //     // 180 degree turn
    //     else if (std::abs(priorDirection - direction) == 2)
    //     {
    //         MMS::turnRight();
    //         MMS::turnRight();
    //         MMS::moveForward();
    //     }
    //     else if ((priorDirection + 1) % 4 == direction)
    //     {
    //         MMS::turnRight();
    //         MMS::moveForward();
    //     }
    //     else
    //     {
    //         MMS::turnLeft();
    //         MMS::moveForward();
    //     }
    // }
}