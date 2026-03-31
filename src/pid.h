#ifndef _PID_H_
#define _PID_H_

#include "motor.h"

#define Kp          3.0
#define Ki          2.0
#define Kd          0.1

class PID {

public:
    PID(Motor* motor): motor(motor) {} 

    void update();
    void setTargetSpeed(double speedMS);
    void setCurrentSpeed(double speedMS) { currentMS = speedMS; }

private:
    Motor* motor;
    uint64_t lastTime = 0;
    double targetMS = 0;
    double currentMS = 0;
    double integral = 0;
    double lastError = 0;
};

#endif