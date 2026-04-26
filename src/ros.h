#ifndef _ROS_H_
#define _ROS_H_

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joy.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/laser_scan.h>

#include <geometry_msgs/msg/twist.h>

#include "rc.h"
#include "imu.h"
#include "drive.h"

#define AGENT_PING_INTERVAL    5000


class ROS {

public:
    void begin();
    void update();

    void publishJoyMessage(const RC& rc);
    void publishBatteryStatusMessage(double voltage, int remainingCapcity);
    void publishImuMessage(const IMU& imu);
    void publishJointState(float leftWheelSpeed, float rightWheelSpeed, int64_t leftEncoderCount, int64_t rightEncoderCount);

    bool isAgentConnected() const { return agentConnected; }

    void handleTwist(const void* msgin);

    double getRequestedLinearSpeed() const { return requestedLinearSpeed; }
    double getRequestedAngularSpeed() const { return requestedAngularSpeed; }

private:
    void tryInitROS();
    void freeROS();
    bool pingAgent();
    void updateHeaderTime(std_msgs__msg__Header& header);
    void setupJointStateMessage();
    void setupJoyMessage();
    void setupImuMessage();

private:
    bool agentConnected = false;
    uint64_t lastAgentPingTime;

    rclc_executor_t executor;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;

    float axes[AXES_SIZE];
    int32_t buttons[AUX_SIZE];
    sensor_msgs__msg__Joy joyMsg;
    bool joyMsgUpdated = false;
    rcl_publisher_t joyPublisher;

    sensor_msgs__msg__BatteryState batteryMsg;
    bool batteryMsgUpdated = false;
    rcl_publisher_t batteryPublisher;

    sensor_msgs__msg__Imu imuMsg;
    bool imuMsgUpdated = false;
    rcl_publisher_t imuPublisher;

    sensor_msgs__msg__JointState jointStateMsg;
    bool jointStateMsgUpdated = false;
    rcl_publisher_t jointStatePublisher;

    rcl_subscription_t subscriber;
    geometry_msgs__msg__Twist twistMsg;

    volatile double requestedLinearSpeed = 0;
    volatile double requestedAngularSpeed = 0;
};

#endif