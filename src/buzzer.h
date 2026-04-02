#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <Arduino.h>

#define BEEP_DURATION    200
#define BEEP_INTERVAL    1000

class Buzzer {

public:
    void begin();
    void update();

    void beep();

    void on();
    void off();

    bool isOn() const { return enabled; }

private:
    bool enabled = false;
    bool oneBeep = false;
    uint64_t lastBeepTime = 0;
};

#endif