#include "movement.h"
#include "robot_config.h"
#include "motors.h"

Movement::Movement(Motors& motors) : motors(motors) {}

void Movement::moveForward() {
    motors.setSpeed(FORWARD_SPEED);
}

void Movement::moveBackward() {
    motors.setSpeed(BACKWARD_SPEED);
}

void Movement::turnLeft() {
    motors.setSpeed(LEFT_TURN_SPEED);
}

void Movement::turnRight() {
    motors.setSpeed(RIGHT_TURN_SPEED);
}

void Movement::stop() {
    motors.stop();
}