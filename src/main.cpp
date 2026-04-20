#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#include "esp_wifi.h"
#include "esp_bt.h"

#include "drive.h"
#include "rc.h"
#include "rc_map.h"
#include "battery.h"
#include "buzzer.h"
#include "imu.h"
#include "ros.h"

#define RC_PUBLISH_INTERVAL          20
#define JOINT_PUBLISH_INTERVAL       10
#define IMU_PUBLISH_INTERVAL         10
#define BATTERY_PUBLISH_INTERVAL     250

RC rc;
Drive drive;
Buzzer buzzer;
IMU imu;
ROS ros;

uint64_t lastRcPublishTime = 0;
uint64_t lastBatteryPublishTime = 0;
uint64_t lastImuPublishTime = 0;
uint64_t lastJointPublishTime = 0;

bool failsafe = false;
bool armed = false;
bool motorsStopped = false;

// Error handle loop
void errorLoop() {
  while(1) {
    digitalWrite(LED_GPIO, LOW);  
    delay(1000);
    digitalWrite(LED_GPIO, HIGH);  
    delay(1000);
  }
}

void disableWifiBT() {
  esp_wifi_stop();
  esp_bt_controller_disable();  
  esp_wifi_deinit();
}

void publishROSMessages() {
  uint64_t currentTime = millis();

  if (currentTime - lastRcPublishTime >= RC_PUBLISH_INTERVAL) {
    rc.sendAttitude(imu.getPitch(), imu.getRoll(), imu.getYaw());
    ros.publishJoyMessage(rc);
    lastRcPublishTime = currentTime;
  }

  if (currentTime - lastBatteryPublishTime >= BATTERY_PUBLISH_INTERVAL) {
    double voltage = getBatteryVoltage();
    int remainingCapacity = getRemainingBatteryCapacity(voltage);
    rc.sendRxBattery(voltage, 0, 0, remainingCapacity);
    ros.publishBatteryStatusMessage(voltage, remainingCapacity);
    lastBatteryPublishTime = currentTime;
  }

  if (currentTime - lastImuPublishTime >= IMU_PUBLISH_INTERVAL) {
    ros.publishImuMessage(imu);
    lastImuPublishTime = currentTime;
  }

  if (currentTime - lastJointPublishTime >= JOINT_PUBLISH_INTERVAL) {
    ros.publishJointState(
      drive.getLeftWheelSpeed(), drive.getRightWheelSpeed(),
      drive.getLeftEncoderCount(), drive.getRightEncoderCount()
    );
    lastJointPublishTime = currentTime;
  }
}

void driveLoop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    for (;;) {
        drive.update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void controlLoop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1000Hz
    for (;;) {
        rc.update();
        buzzer.update();
        imu.update();
        ros.update();

        armed = rc.getButtonState(RC_BTN_SA) == BTN_STATE_PRESSED;

        if (rc.isLinkUp()) {
          if (failsafe) {
            buzzer.off();
          }

          failsafe = false;

          if (armed) {
            motorsStopped = false;
            float throttle = rc.getChannel(RC_PITCH);
            float steering = rc.getChannel(RC_ROLL);
            drive.setSpeedInPct(throttle, steering);
          } else if (!motorsStopped) {
            motorsStopped = true;
            drive.stop();
          }
        } else if (!failsafe) {
          failsafe = true;
          motorsStopped = true;
          drive.stop();
          buzzer.on();
        }

        publishROSMessages();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
  disableWifiBT();

  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, LOW);  
  
  rc.begin();
  drive.begin();
  buzzer.begin();

  if (!imu.begin()) errorLoop();

  delay(1000);
  imu.calibrate();

  ros.begin();

  buzzer.beep();

  xTaskCreatePinnedToCore(
      driveLoop,
      "DriveTask",
      4096, 
      NULL,
      3,
      NULL,
      0 
  );

    xTaskCreatePinnedToCore(
      controlLoop,
      "ControlTask",
      4096, 
      NULL,
      3,
      NULL,
      1 
  );
}

void loop() {
}
