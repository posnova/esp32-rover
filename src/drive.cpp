#include "drive.h"
#include <Arduino.h>

const double WHEEL_L = WHEEL_DIAMETER * PI;
const double DISTANCE_PER_PULSE = WHEEL_L / CPR;

const double MIN_SPEED = 0.05;
const double MAX_SPEED = 0.3;
const double MAX_ANGULAR = 3.8;

Drive::Drive(): motorLeft(MOTOR_LEFT), motorRight(MOTOR_RIGHT), pidLeft(&motorLeft), pidRight(&motorRight)
{

}

void Drive::begin() {
    Motor::init();
    stop();
}

void Drive::stop() {
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
}

void Drive::setSpeedInPct(double throttle, double steering) {
    setSpeed(MAX_SPEED * throttle, MAX_ANGULAR * steering);
}

double Drive::limitSpeed(double speedMS) {
    if (speedMS != 0) {
        if (abs(speedMS) < MIN_SPEED) {
            if (speedMS < 0)
                speedMS = -MIN_SPEED;
            else 
                speedMS = MIN_SPEED;
        }
    }
    return speedMS;
}

void Drive::setSpeed(double linearMS, double angularRadS) {
    pidLeft.setTargetSpeed(limitSpeed(linearMS + (angularRadS * (TRACK_WIDTH / 2.0))));
    pidRight.setTargetSpeed(limitSpeed(linearMS - (angularRadS * (TRACK_WIDTH / 2.0))));
}

double Drive::calculateSpeed(int64_t count, int64_t lastCount, double dt) {
    int64_t diff = count - lastCount; 
    double dtSeconds = dt / 1000.0;
    if (dtSeconds <= 0) return 0;
    return (diff * DISTANCE_PER_PULSE) / dtSeconds;
}

void Drive::update() {
    uint64_t now = millis();
    uint64_t dt = now - lastMeasureTime;

    if (dt >= UPDATE_INTERVAL) {
        lastMeasureTime = now;

        int64_t countLeft = motorLeft.getPulseCount();
        int64_t countRight = motorRight.getPulseCount();
        
        leftWheelSpeed = calculateSpeed(countLeft, lastLeftEncoderCount, dt);
        rightWheelSpeed = calculateSpeed(countRight, lastRightEncoderCount, dt);

        lastLeftEncoderCount = countLeft;
        lastRightEncoderCount = countRight;
        
        pidLeft.setCurrentSpeed(leftWheelSpeed);
        pidRight.setCurrentSpeed(rightWheelSpeed);

        pidLeft.update();
        pidRight.update();
    }
}