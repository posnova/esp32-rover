#ifndef __MOTOR_H__
#define __MOTOR_H__

#define PWM_GPIO_M1A              4
#define PWM_GPIO_M1B              5

#define PWM_GPIO_M2A              15
#define PWM_GPIO_M2B              16

#define PWM_GPIO_M3A              9
#define PWM_GPIO_M3B              10

#define PWM_GPIO_M4A              13
#define PWM_GPIO_M4B              14

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

void motors_init();
void motor_set_speed(int motor, double speed);

class Motor {
public:
    Motor(int motor) : motor(motor) {};
    static void init();

    void update();
    void set_speed(double speed);
    
private:
    const int motor;
    unsigned long last_update = 0;
    double current_speed = 0;
    double target_speed = 0;
};


#endif