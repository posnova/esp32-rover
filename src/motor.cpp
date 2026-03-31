#include <Arduino.h>
#include "motor.h"
#include "gpio.h"


void set_pwm(int a, int b, int pwm, bool reverse) {
    if (reverse) {
        ledcWrite(a, 0);
        ledcWrite(b, pwm);
    } else {
        ledcWrite(a, pwm);
        ledcWrite(b, 0);
    }
}

void Motor::init() {
  ledcSetup(M1A, PWM_MOTOR_FREQ_HZ, PWM_MOTOR_RESOLUTION);
  ledcSetup(M1B, PWM_MOTOR_FREQ_HZ, PWM_MOTOR_RESOLUTION);

  ledcSetup(M2A, PWM_MOTOR_FREQ_HZ, PWM_MOTOR_RESOLUTION);
  ledcSetup(M2B, PWM_MOTOR_FREQ_HZ, PWM_MOTOR_RESOLUTION);

  ledcAttachPin(PWM_GPIO_M1A, M1A);
  ledcAttachPin(PWM_GPIO_M1B, M1B);

  ledcAttachPin(PWM_GPIO_M2A, M2A);
  ledcAttachPin(PWM_GPIO_M2B, M2B);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
}

Motor::Motor(int motorId): motorId(motorId) {
    if (motorId == MOTOR_LEFT) {
        encoder.attachHalfQuad(ENC_GPIO_H2A, ENC_GPIO_H2B);
    } else {
        encoder.attachHalfQuad(ENC_GPIO_H1A, ENC_GPIO_H1B);
    }
    encoder.setCount(0);
}

void Motor::setSpeed(double speed) {
    targetSpeed = speed;
}

int64_t Motor::getPulseCount() { 
    if (motorId == MOTOR_LEFT)
        return -1 * encoder.getCount(); 
    return encoder.getCount(); 
}

void Motor::update() {
    unsigned long now = millis();
    if (now - lastUpdate < LOOP_INTERVAL) return;
    lastUpdate = now;

    if (currentSpeed < targetSpeed) {
        currentSpeed += RAMP_STEP;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    } else if (currentSpeed > targetSpeed) {
        currentSpeed -= RAMP_STEP;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }

    int current_pwm = 0;
    if (abs(currentSpeed) > 0.01) { // avoid jitter at 0
        current_pwm = PWM_DEADBAND + (PWM_MAX - PWM_DEADBAND) * min(1.0, abs(currentSpeed));
    }
    bool reverse = currentSpeed < 0;

    switch (motorId) 
    {
    case MOTOR_RIGHT:
        set_pwm(M1A, M1B, current_pwm, reverse);
        break;
    case MOTOR_LEFT:
        set_pwm(M2B, M2A, current_pwm, reverse);
        break;
    }
}