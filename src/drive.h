#ifndef _DRIVE_H_
#define _DRIVE_H_

#include "motor.h"
#include "pid.h"

// encoder's pulses per revolution
#define ENC_PPR          11

// 520 motor ratio is 45:1
#define MOTOR_RATIO      45

#define WHEEL_DIAMETER   0.047

// counts per revolution
#define CPR (ENC_PPR * MOTOR_RATIO * 2) 

// distance between center of the wheels, in meters
#define TRACK_WIDTH      0.154

#define UPDATE_INTERVAL 10

class Drive {

public:
    Drive();
    void begin();

    void setSpeedInPct(double throttle, double steering);
    void setSpeed(double linearMS, double angularRadS);
    void update();
    void stop();

    int64_t getLeftEncoderCount() const { return lastLeftEncoderCount; }
    int64_t getRightEncoderCount() const { return lastRightEncoderCount; }
    double getLeftWheelSpeed() const { return leftWheelSpeed; }
    double getRightWheelSpeed() const { return rightWheelSpeed; }

private:
    double calculateSpeed(int64_t count, int64_t lastCount, double dt);
    double limitSpeed(double speedMS);

private:
    Motor motorLeft, motorRight;
    PID pidLeft, pidRight;
    uint64_t lastMeasureTime = 0;
    int64_t lastLeftEncoderCount;
    int64_t lastRightEncoderCount;
    double leftWheelSpeed = 0;
    double rightWheelSpeed = 0;
};

#endif