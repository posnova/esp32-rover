#include "imu.h"
#include "gpio.h"

bool IMU::begin() {
    Wire.begin(IMU_SDA_GPIO, IMU_SCL_GPIO);

    if (icm.begin() != 0) return false;
    icm.startAccel(100, 16);
    icm.startGyro(100, 2000);
    delay(100);
    lastTime = micros();
    return true;
}

void IMU::calibrate() {
    const int samples = 500;
    gyroOffset[0] = 0;
    gyroOffset[1] = 0;
    gyroOffset[2] = 0;

    for (int i = 0; i < samples; i++) {
        icm.getDataFromRegisters(event);
        gyroOffset[0] += event.gyro[0];
        gyroOffset[1] += event.gyro[1];
        gyroOffset[2] += event.gyro[2];
        delay(10);
    }

    gyroOffset[0] = gyroOffset[0] / samples;
    gyroOffset[1] = gyroOffset[1] / samples;
    gyroOffset[2] = gyroOffset[2] / samples;
}

void IMU::updateAttitude(float dt, float ax, float ay, float az, float gx, float gy, float gz) {
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az));
    float rollAcc = atan2(ay, az);
    
    // Complementary Filter
    attitude[0] = 0.98f * (attitude[0] + gy * dt) + 0.02f * pitchAcc;
    attitude[1] = 0.98f * (attitude[1] + gx * dt) + 0.02f * rollAcc;
    
    // Yaw (Gyro only - will drift)
    attitude[2] += gz * dt;
    // Keep Yaw within -PI to +PI range
    if (attitude[2] > M_PI)  attitude[2] -= 2.0f * M_PI;
    if (attitude[2] < -M_PI) attitude[2] += 2.0f * M_PI;
}

void IMU::updateIMUMessage(sensor_msgs__msg__Imu& msg) const {
    // Convert Euler angles (attitude) to Quaternion
    // Assuming attitude[0] = Pitch, attitude[1] = Roll, attitude[2] = Yaw
    float p = attitude[0];
    float r = attitude[1];
    float y = attitude[2];

    double cy = cos(y * 0.5);
    double sy = sin(y * 0.5);
    double cp = cos(p * 0.5);
    double sp = sin(p * 0.5);
    double cr = cos(r * 0.5);
    double sr = sin(r * 0.5);

    msg.orientation.w = cr * cp * cy + sr * sp * sy;
    msg.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.orientation.z = cr * cp * sy - sr * sp * cy;

    // Fill Angular Velocity (Rad/s)
    // Using the last processed values from 'event' and 'gyroOffset'
    const float degToRad = M_PI / 180.0f;
    msg.angular_velocity.x = ((event.gyro[0] - gyroOffset[0]) / 16.4f) * degToRad;
    msg.angular_velocity.y = ((event.gyro[1] - gyroOffset[1]) / 16.4f) * degToRad;
    msg.angular_velocity.z = ((event.gyro[2] - gyroOffset[2]) / 16.4f) * degToRad;

    // Fill Linear Acceleration (m/s^2)
    // Convert G-force to m/s^2 (1G = 9.80665 m/s^2)
    const float gToMs2 = 9.80665f;
    msg.linear_acceleration.x = (event.accel[0] / 2048.0f) * gToMs2;
    msg.linear_acceleration.y = (event.accel[1] / 2048.0f) * gToMs2;
    msg.linear_acceleration.z = (event.accel[2] / 2048.0f) * gToMs2;
}

void IMU::update() {
    uint64_t currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f; // in seconds

    if (dt >= 0.01f) { // run at 100Hz
        lastTime = currentTime;
        icm.getDataFromRegisters(event);

        // Convert Raw Accel to G-force (Ratios for atan2)
        // Range 16g = 2048 LSB/g
        float ax = event.accel[0] / 2048.0f;
        float ay = event.accel[1] / 2048.0f;
        float az = event.accel[2] / 2048.0f;

        // Convert Raw Gyro to Radians/sec
        // Range 2000dps = 16.4 LSB/dps. Then convert deg to rad.
        const float degToRad = M_PI / 180.0f;
        float gx = ((event.gyro[0] - gyroOffset[0]) / 16.4f) * degToRad;
        float gy = ((event.gyro[1] - gyroOffset[1]) / 16.4f) * degToRad;
        float gz = ((event.gyro[2] - gyroOffset[2]) / 16.4f) * degToRad;

        updateAttitude(dt, ax, ay, az, gx, gy, gz);
    }
}