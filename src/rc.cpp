#include "rc.h"

void RC::begin() {
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, JOY_GPIO_RX, JOY_GPIO_TX);
  crsf.begin(crsfSerial);
}

void RC::update() {
  crsf.update();
}

double RC::getChannel(int ch) const {
  return (crsf.getChannel(ch) - 1500) / 500.0; 
}

int RC::getChannelRaw(int ch) const {
  return crsf.getChannel(ch);
}

int RC::getButtonState(int ch) const {
  int raw = getChannelRaw(ch);
  if (raw < 1100) return BTN_STATE_RELEASED;
  if (raw > 1400 & raw < 1600) return BTN_STATE_CENTER;
  if (raw > 1900) return BTN_STATE_PRESSED;
  return BTN_STATE_UNKNOWN;
}

void RC::sendRxBattery(float voltage, float current, float capacity, float remaining) {
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;   //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

void RC::sendAttitude(float pitch, float roll, float yaw) {
  crsf_sensor_attitude_t crsfAttitude = { 0 };

  // Values are MSB first (BigEndian)
  crsfAttitude.pitch = htobe16((uint16_t)(pitch*10000.0));
  crsfAttitude.roll = htobe16((uint16_t)(roll*10000.0));
  crsfAttitude.yaw = htobe16((uint16_t)(yaw*10000.0));
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_ATTITUDE, &crsfAttitude, sizeof(crsfAttitude));
}