#include "buzzer.h"
#include "gpio.h"

void Buzzer::begin() {
    pinMode(BUZZER_GPIO, OUTPUT);
    off();
}

void Buzzer::update() {
    if (!enabled && !oneBeep) return;

    uint64_t currentTime = millis();
    uint64_t deltaTime = currentTime - lastBeepTime;

    if (deltaTime >= BEEP_INTERVAL + BEEP_DURATION) {
        digitalWrite(BUZZER_GPIO, HIGH);
        lastBeepTime = currentTime;
    } else if (deltaTime >= BEEP_DURATION) {
        digitalWrite(BUZZER_GPIO, LOW);
        oneBeep = false;
    }
}

void Buzzer::beep() {
    oneBeep = true;
    lastBeepTime = -INT_MAX;
}

void Buzzer::on() {
    enabled = true;
}

void Buzzer::off() {
    enabled = false;
    digitalWrite(BUZZER_GPIO, LOW);
}
