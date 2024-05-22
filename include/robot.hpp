#ifndef ROBOT_HPP
#define ROBOT_HPP

class Robot {
    public:
        bool wallFront();
        bool wallRight();
        bool wallLeft();

        void moveForward(int distance = 1);
        void turnRight();
        void turnLeft();
};

#endif