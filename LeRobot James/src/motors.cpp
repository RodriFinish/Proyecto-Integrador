#include <Arduino.h>

#include "motors.h"
#include "robot_config.h"

Motors::Motors()
    : ain1Pin(TB6612_AIN1_PIN), ain2Pin(TB6612_AIN2_PIN),
      pwmaPin(TB6612_PWMA_PIN), bin1Pin(TB6612_BIN1_PIN),
      bin2Pin(TB6612_BIN2_PIN), pwmbPin(TB6612_PWMB_PIN),
      standbyPin(TB6612_STBY_PIN) {}

void Motors::begin() {
    pinMode(ain1Pin, OUTPUT);
    pinMode(ain2Pin, OUTPUT);
    pinMode(pwmaPin, OUTPUT);
    pinMode(bin1Pin, OUTPUT);
    pinMode(bin2Pin, OUTPUT);
    pinMode(pwmbPin, OUTPUT);
    pinMode(standbyPin, OUTPUT);

    analogWrite(pwmaPin, 0);
    analogWrite(pwmbPin, 0);
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, LOW);
    digitalWrite(bin1Pin, LOW);
    digitalWrite(bin2Pin, LOW);
    digitalWrite(standbyPin, HIGH);
}

void Motors::setSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    digitalWrite(ain1Pin, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(ain2Pin, leftSpeed < 0 ? HIGH : LOW);
    digitalWrite(bin1Pin, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(bin2Pin, rightSpeed < 0 ? HIGH : LOW);

    analogWrite(pwmaPin, abs(leftSpeed));
    analogWrite(pwmbPin, abs(rightSpeed));
}

void Motors::stop() {
    analogWrite(pwmaPin, 0);
    analogWrite(pwmbPin, 0);
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, LOW);
    digitalWrite(bin1Pin, LOW);
    digitalWrite(bin2Pin, LOW);
}