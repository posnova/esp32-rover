#include "battery.h"
#include "gpio.h"
#include <Arduino.h>

double getBatteryVoltage() {
    uint16_t mV = analogReadMilliVolts(BATTERY_GPIO);
    return mV / 1000.0 / 3.3 * MAX_BATTERY_VOLTAGE * 1.0488;
}

int getRemainingBatteryCapacity(double voltage) {
    return (constrain(voltage, MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE) - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100;
}