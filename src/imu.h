#ifndef _IMU_H_
#define _IMU_H_

#include "ICM42670P.h"
#include <sensor_msgs/msg/imu.h>

class IMU {
public:
    IMU(): icm(Wire, 0) {}
    bool begin();
    void update();

    void calibrate();

    float getPitch() const { return attitude[0]; }
    float getRoll() const { return attitude[1]; }
    float getYaw() const { return attitude[2]; }

    void updateIMUMessage(sensor_msgs__msg__Imu& msg) const;

private:
    void updateAttitude(float dt, float ax, float ay, float az, float gx, float gy, float gz);

private:
    uint64_t lastTime = 0;
    ICM42670 icm;
    inv_imu_sensor_event_t event;
    float attitude[3] = {0};
    int16_t gyroOffset[3] = {0};
};


#endif