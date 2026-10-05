#ifndef MOTORS_H
#define MOTORS_H

class Motors {
public:
    Motors();
    void begin();
    void setSpeed(int leftSpeed, int rightSpeed);
    void stop();

private:
    int ain1Pin;
    int ain2Pin;
    int pwmaPin;
    int bin1Pin;
    int bin2Pin;
    int pwmbPin;
    int standbyPin;
};

#endif
