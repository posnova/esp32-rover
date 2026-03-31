#include "drive.h"
#include <Arduino.h>

const double WHEEL_L = WHEEL_DIAMETER * PI;
const double DISTANCE_PER_PULSE = WHEEL_L / CPR;

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

void Drive::moveInPct(double throttle, double steering) {
    setSpeed(MAX_SPEED * throttle, MAX_ANGULAR * steering);
}

void Drive::setSpeed(double linearMS, double angularRadS) {
    pidLeft.setTargetSpeed(linearMS + (angularRadS * (TRACK_WIDTH / 2.0)));
    pidRight.setTargetSpeed(linearMS - (angularRadS * (TRACK_WIDTH / 2.0)));
}

double Drive::calculateSpeed(int64_t count, int64_t lastCount, double dt) {
    int64_t diff = abs(count) - abs(lastCount);
    return diff * DISTANCE_PER_PULSE / (dt / 1000.0);
}

void Drive::update() {
    motorLeft.update();
    motorRight.update();

    uint64_t now = millis();
    uint64_t dt = now - lastMeasureTime;

    if (dt >= SPEED_MEASURE_INTERVAL) {
        lastMeasureTime = now;
        
        leftWheelSpeed = calculateSpeed(motorLeft.getPulseCount(), lastLeftEncoderCount, dt);
        rightWheelSpeed = calculateSpeed(motorRight.getPulseCount(), lastRightEncoderCount, dt);

        lastLeftEncoderCount = motorLeft.getPulseCount();
        lastRightEncoderCount = motorRight.getPulseCount();
        
        pidLeft.setCurrentSpeed(leftWheelSpeed);
        pidRight.setCurrentSpeed(rightWheelSpeed);

        pidLeft.update();
        pidRight.update();
    }
}