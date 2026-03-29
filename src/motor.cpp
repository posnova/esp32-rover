#include <Arduino.h>
#include "motor.h"


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
}


void Motor::set_speed(double speed) {
    target_speed = speed;
}

void Motor::update() {
    unsigned long now = millis();
    if (now - last_update < LOOP_INTERVAL) return;
    last_update = now;

    if (current_speed < target_speed) {
        current_speed += RAMP_STEP;
        if (current_speed > target_speed) current_speed = target_speed;
    } else if (current_speed > target_speed) {
        current_speed -= RAMP_STEP;
        if (current_speed < target_speed) current_speed = target_speed;
    }

    int current_pwm = 0;
    if (abs(current_speed) > 0.01) { // Small epsilon to avoid jitter at 0
        current_pwm = PWM_DEADBAND + (PWM_MAX - PWM_DEADBAND) * min(1.0, abs(current_speed));
    }
    bool reverse = current_speed < 0;

    switch (motor) 
    {
    case MOTOR_RIGHT:
        set_pwm(M1A, M1B, current_pwm, reverse);
        break;
    case MOTOR_LEFT:
        set_pwm(M2B, M2A, current_pwm, reverse);
        break;
    }
}