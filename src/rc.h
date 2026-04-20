#ifndef _RC_H_
#define _RC_H_

#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include "gpio.h"

#define AXES_SIZE     4
#define AUX_SIZE      12

#define BTN_STATE_UNKNOWN  -1
#define BTN_STATE_RELEASED  0
#define BTN_STATE_PRESSED   1
#define BTN_STATE_CENTER    2


class RC {

public:
    RC(): crsfSerial(JOY_SERIAL) {}
    void begin();
    void update();
    bool isLinkUp() const { return crsf.isLinkUp(); }

    double getChannel(int ch) const;
    int getChannelRaw(int ch) const;

    int getButtonState(int ch) const;

    void sendRxBattery(float voltage, float current, float capacity, float remaining);
    void sendAttitude(float pitch, float roll, float yaw);

private:
    HardwareSerial crsfSerial;
    AlfredoCRSF crsf;
};

#endif