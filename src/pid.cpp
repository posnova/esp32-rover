#include "pid.h"
#include <Arduino.h>

void PID::setTargetSpeed(double speedMS) {
    targetMS = speedMS;
    if (speedMS == 0) {
        integral = 0;
    }
}

void PID::update() {
    if (abs(targetMS) < 0.001) { 
        motor->setSpeed(0);
        integral = 0;
        lastError = 0;
        return;
    }

    uint64_t currentTime = millis();

    if (lastTime == 0) {
        lastTime = currentTime;
        return;
    }

    double dt = (currentTime - lastTime) / 1000.0;

    double error = targetMS - currentMS;

    integral += error * dt;
    integral = constrain(integral, -0.2, 0.2);

    double derivative = (error - lastError) / dt;
    double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    output = constrain(output, -1.0, 1.0);

    motor->setSpeed(output);

    // Serial.print("Motor ");
    // Serial.print(motor->getMotorId());
    // Serial.print(" t: ");
    // Serial.print(targetMS);
    // Serial.print(" c: ");
    // Serial.print(currentMS);
    // Serial.print(" e: ");
    // Serial.print(error);
    // Serial.print(" i: ");
    // Serial.print(integral);
    // Serial.print(" d: ");
    // Serial.print(derivative);
    // Serial.print(" o: ");
    // Serial.println(output);

    lastError = error;
    lastTime = currentTime;
}