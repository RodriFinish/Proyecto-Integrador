#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// TB6612FNG connections to Arduino Nano.
// Motor A drives the left wheel; motor B drives the right wheel.
#define TB6612_AIN1_PIN 5
#define TB6612_AIN2_PIN 6
#define TB6612_PWMA_PIN 3
#define TB6612_BIN1_PIN 10
#define TB6612_BIN2_PIN 11
#define TB6612_PWMB_PIN 9
#define TB6612_STBY_PIN 4

// Motor speeds
#define FORWARD_SPEED 200, 200
#define BACKWARD_SPEED -150, -150
#define LEFT_TURN_SPEED -150, 200
#define RIGHT_TURN_SPEED 200, -150

#define MAX_SPEED 255
#define MIN_SPEED 0

#define ROBOT_WIDTH 20 // cm
#define ROBOT_LENGTH 20 // cm
#define ROBOT_HEIGHT 15 // cm

// Add any additional configuration constants as needed.

#endif