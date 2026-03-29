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

#include "motor.h"


// Set up a new Serial object
HardwareSerial crsfSerial(2);
AlfredoCRSF crsf;

// // micro-ROS objects
// rcl_publisher_t publisher;
// sensor_msgs__msg__Joy msg;
// rclc_executor_t executor;
// rcl_node_t node;
// rcl_allocator_t allocator;
// rclc_support_t support;

Motor motorLeft(MOTOR_LEFT);
Motor motorRight(MOTOR_RIGHT);

#define LED_PIN 13

#define AXES_SIZE 4
#define AUX_SIZE 12

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, LOW);  
    delay(1000);
    digitalWrite(LED_PIN, HIGH);  
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

static void sendRxBattery(float voltage, float current, float capacity, float remaining) {
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;   //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

static void sendAttitude(float pitch, float roll, float yaw) {
  crsf_sensor_attitude_t crsfAttitude = { 0 };

  // Values are MSB first (BigEndian)
  crsfAttitude.pitch = htobe16((uint16_t)(pitch*10000.0));
  crsfAttitude.roll = htobe16((uint16_t)(roll*10000.0));
  crsfAttitude.yaw = htobe16((uint16_t)(yaw*10000.0));
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_ATTITUDE, &crsfAttitude, sizeof(crsfAttitude));
}

// void disableWifiBT() {
//   esp_wifi_stop();
//   esp_bt_controller_disable();  
//   esp_wifi_deinit();
// }

void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, 35, 36);
  if (!crsfSerial) error_loop();

  crsf.begin(crsfSerial);

  // set_microros_transports();
  //disableWifiBT();
 Motor::init();

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


  // pinMode(PWM_GPIO_M1A, OUTPUT);
  // pinMode(PWM_GPIO_M1B, OUTPUT);
  // pinMode(PWM_GPIO_M2A, OUTPUT);
  // pinMode(PWM_GPIO_M2B, OUTPUT);

 motorLeft.set_speed(0);
 motorRight.set_speed(0);
}

uint64_t last_publish = 0;

void loop() {
    crsf.update();

    if (crsf.isLinkUp()) {
// Mapping 1000-2000 range to -1.0 to 1.0
  float throttle = (crsf.getChannel(2) - 1500) / 500.0; 
  float steering = (crsf.getChannel(1) - 1500) / 500.0;

  // Simple Differential Drive Mix
  motorLeft.set_speed(throttle + steering);
  motorRight.set_speed(throttle - steering);

    } else {
        motorLeft.set_speed(0);
  motorRight.set_speed(0);
    }



   motorLeft.update();
   motorRight.update();



if (millis() - last_publish > 25) {
      last_publish = millis();
           sendAttitude(0, 1, 2);
     sendRxBattery(12.6, 1.2, 50, 100);
    }

    // initJoyMessage();

    // if (millis() - last_publish > 10) {
    //   last_publish = millis();
    //   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // }
}
