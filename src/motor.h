#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <ESP32Encoder.h>

#define PWM_MOTOR_RESOLUTION      8
#define PWM_MOTOR_FREQ_HZ         25000

#define M1A                       0
#define M1B                       1
#define M2A                       2
#define M2B                       3

#define MOTOR_RIGHT               1
#define MOTOR_LEFT                2

#define PWM_MIN                   0
#define PWM_MAX                   255
#define PWM_DEADBAND              130

#define LOOP_INTERVAL             10

#define RAMP_STEP                 0.05


class Motor {
public:
    Motor(int motorId);
    static void init();

    void update();
    void setSpeed(double speed);

    int64_t getPulseCount();

    int getMotorId() { return motorId; }
    
private:
    const int motorId;
    ESP32Encoder encoder;
    unsigned long lastUpdate = 0;
    double currentSpeed = 0;
    double targetSpeed = 0;
};


#endif