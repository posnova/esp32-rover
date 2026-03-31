#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#include "esp_wifi.h"
#include "esp_bt.h"

// #include <micro_ros_arduino.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <sensor_msgs/msg/joy.h>

#include "drive.h"
#include "rc.h"
#include "rc_map.h"
#include "battery.h"

RC rc;
Drive drive;

// // micro-ROS objects
// rcl_publisher_t publisher;
// sensor_msgs__msg__Joy msg;
// rclc_executor_t executor;
// rcl_node_t node;
// rcl_allocator_t allocator;
// rclc_support_t support;


#define AXES_SIZE 4
#define AUX_SIZE 12

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_GPIO, LOW);  
    delay(1000);
    digitalWrite(LED_GPIO, HIGH);  
    delay(1000);
  }
}

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// void initJoyMessage() {
//   int v = 0;
//   for (int ch = 1; ch <= CRSF_NUM_CHANNELS; ch++) {
//     v = crsf.getChannel(ch);
//     if (ch <= AXES_SIZE) {
//       msg.axes.data[ch - 1] = map(v, 1000, 2000, -1000, 1000) / 1000.0;
//     } else {
//       msg.buttons.data[ch - 5] = v;
//     }
//   }
// }


void disableWifiBT() {
  esp_wifi_stop();
  esp_bt_controller_disable();  
  esp_wifi_deinit();
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, HIGH);  
  
  rc.begin();
  drive.begin();

  // set_microros_transports();
  disableWifiBT();

  //delay(2000);

  // allocator = rcl_get_default_allocator();

  // //create init_options
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // // create node
  // RCCHECK(rclc_node_init_default(&node, "esp32_joystick", "", &support));

  
  // // Init Publisher
  // RCCHECK(rclc_publisher_init_default(
  //   &publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
  //   "joy")
  // );


  // // Initialize Joy message memory
  // static float axes[8];
  // static int32_t buttons[8];
  // msg.axes.capacity = AXES_SIZE;
  // msg.axes.size = AXES_SIZE;
  // msg.axes.data = axes;
  // msg.buttons.capacity = AUX_SIZE;
  // msg.buttons.size = AUX_SIZE;
  // msg.buttons.data = buttons;
}

uint64_t last_publish = 0;

void loop() {
    rc.update();
    drive.update();

    if (rc.isLinkUp()) {
      float throttle = rc.getChannel(RC_PITCH);
      float steering = rc.getChannel(RC_ROLL);
      drive.setSpeedInPct(throttle, steering);
    } else {
      drive.stop();
    }


    if (millis() - last_publish > 25) {
        double voltage = get_battery_voltage();
        last_publish = millis();
        rc.sendAttitude(0, 1, 2);
        rc.sendRxBattery(voltage, 0, 0, get_remaining_capacity(voltage));
    }

    // initJoyMessage();

    // if (millis() - last_publish > 10) {
    //   last_publish = millis();
    //   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // }
}
