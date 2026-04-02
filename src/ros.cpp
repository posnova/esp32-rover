#include "ros.h"

void ROS::begin() {
    setupJoyMessage();
    setupJointStateMessage();
    setupImuMessage();

    tryInitROS();
}

void ROS::setupJointStateMessage() {
    jointStateMsg.header.frame_id.data = (char*)"base_link";
    jointStateMsg.header.frame_id.size = strlen(jointStateMsg.header.frame_id.data);

    static char* joint_names[] = {(char*)"left_wheel_joint", (char*)"right_wheel_joint"};
    jointStateMsg.name.data = (rosidl_runtime_c__String*) malloc(2 * sizeof(rosidl_runtime_c__String));
    jointStateMsg.name.size = 2;
    jointStateMsg.name.capacity = 2;

    for(int i = 0; i < 2; i++) {
        jointStateMsg.name.data[i].data = joint_names[i];
        jointStateMsg.name.data[i].size = strlen(joint_names[i]);
        jointStateMsg.name.data[i].capacity = strlen(joint_names[i]) + 1;
    }

    jointStateMsg.position.data = (double*) malloc(2 * sizeof(double));
    jointStateMsg.position.size = 2;
    
    jointStateMsg.velocity.data = (double*) malloc(2 * sizeof(double));
    jointStateMsg.velocity.size = 2;
}

void ROS::setupImuMessage() {
    imuMsg.header.frame_id.data = (char*)"imu_link";
    imuMsg.header.frame_id.size = strlen(imuMsg.header.frame_id.data);
}

void ROS::setupJoyMessage() {
    joyMsg.header.frame_id.data = (char*)"joy";
    joyMsg.header.frame_id.size = strlen(joyMsg.header.frame_id.data);
    joyMsg.axes.capacity = AXES_SIZE;
    joyMsg.axes.size = AXES_SIZE;
    joyMsg.axes.data = axes;
    joyMsg.buttons.capacity = AUX_SIZE;
    joyMsg.buttons.size = AUX_SIZE;
    joyMsg.buttons.data = buttons;
}

bool ROS::pingAgent() {
    return rmw_uros_ping_agent(50, 2) == RCL_RET_OK;
}

void ROS::tryInitROS() {
    set_microros_transports();
    delay(100); 
    Serial.begin(921600); //override hardcoded 115200 in micro_ros_arduino

    delay(500);

    if (!pingAgent()) return;

    allocator = rcl_get_default_allocator();

    //create init_options
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    return;
    }

    // create node
    rclc_node_init_default(&node, "esp32_node", "", &support);

    // Init Publishers
    rclc_publisher_init_default(
    &joyPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "joy"
    );

    rclc_publisher_init_default(
    &batteryPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "battery"
    );

    rclc_publisher_init_default(
    &imuPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"
    );

    rclc_publisher_init_default(
    &jointStatePublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"
    );

    rmw_uros_sync_session(1000);

    agentConnected = true;
    lastAgentPingTime = millis();
}

void ROS::freeROS() {
    if (!agentConnected) return;

    rcl_publisher_fini(&joyPublisher, &node);
    rcl_publisher_fini(&batteryPublisher, &node);
    rcl_publisher_fini(&imuPublisher, &node);
    rcl_publisher_fini(&jointStatePublisher, &node);
    
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    agentConnected = false;
}

void ROS::updateHeaderTime(std_msgs__msg__Header& header) {
    int64_t time_ns = rmw_uros_epoch_nanos();
    header.stamp.sec = (int32_t)(time_ns / 1000000000);
    header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);
}

void ROS::publishJoyMessage(const RC& rc) {
  int v = 0;
  for (int ch = 1; ch <= CRSF_NUM_CHANNELS; ch++) {
    v = rc.getChannelRaw(ch);
    if (ch <= AXES_SIZE) {
      joyMsg.axes.data[ch - 1] = map(v, 1000, 2000, -1000, 1000) / 1000.0;
    } else {
      joyMsg.buttons.data[ch - 5] = v;
    }
  }
  joyMsgUpdated = true;
}

void ROS::publishBatteryStatusMessage(double voltage, int remainingCapcity) {
    batteryMsg.voltage = voltage;
    batteryMsg.percentage = remainingCapcity;
    batteryMsg.present = true;
    batteryMsgUpdated = true;
}

void ROS::publishImuMessage(const IMU& imu) {
    imu.updateIMUMessage(imuMsg);
    imuMsgUpdated = true;
}

void ROS::publishJointState(
    float leftWheelSpeed, float rightWheelSpeed, int64_t leftEncoderCount, int64_t rightEncoderCount
) {
    const float wheelRadius = WHEEL_DIAMETER / 2.0;

    // Convert linear speed (m/s) to angular velocity (rad/s)
    // ω = v / r
    jointStateMsg.velocity.data[0] = leftWheelSpeed / wheelRadius;
    jointStateMsg.velocity.data[1] = rightWheelSpeed / wheelRadius;

    jointStateMsg.position.data[0] = (double)leftEncoderCount * (2.0f * M_PI / CPR);
    jointStateMsg.position.data[1] = (double)rightEncoderCount * (2.0f * M_PI / CPR);

    jointStateMsgUpdated = true;
}

void ROS::update() {
    uint64_t currentTime = millis();

    if (!agentConnected) {
        tryInitROS();
        return;
    } else if (currentTime - lastAgentPingTime >= AGENT_PING_INTERVAL) {
        if (!pingAgent()) {
            freeROS();
            return;
        }
        lastAgentPingTime = currentTime;
    }

    if (joyMsgUpdated) {  
        updateHeaderTime(joyMsg.header);
        if (rcl_publish(&joyPublisher, &joyMsg, NULL) == RCL_RET_OK)
            joyMsgUpdated = false;
    }

    if (batteryMsgUpdated) {  
        updateHeaderTime(batteryMsg.header);
        if (rcl_publish(&batteryPublisher, &batteryMsg, NULL) == RCL_RET_OK)
            batteryMsgUpdated = false;
    }

    if (imuMsgUpdated) {
        updateHeaderTime(imuMsg.header);
        if (rcl_publish(&imuPublisher, &imuMsg, NULL) == RCL_RET_OK)
            imuMsgUpdated = false;
    }

    if (jointStateMsgUpdated) {
        updateHeaderTime(jointStateMsg.header);
        if (rcl_publish(&jointStatePublisher, &jointStateMsg, NULL) == RCL_RET_OK)
            jointStateMsgUpdated = false;
    }
}