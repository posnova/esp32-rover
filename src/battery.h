#ifndef _BATTERY_H_
#define _BATTERY_H_

#define MIN_BATTERY_VOLTAGE  9.0
#define MAX_BATTERY_VOLTAGE  12.6

double get_battery_voltage();
int get_remaining_capacity(double voltage);

#endif