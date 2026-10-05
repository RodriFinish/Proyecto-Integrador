#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "motors.h"

class Movement {
public:
    explicit Movement(Motors& motors);
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
    void stop();

private:
    Motors& motors;
};

#endif