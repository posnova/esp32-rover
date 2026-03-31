#ifndef _RC_H_
#define _RC_H_

#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include "gpio.h"

class RC {

public:
    RC(): crsfSerial(JOY_SERIAL) {}
    void begin();
    void update();
    bool isLinkUp() { return crsf.isLinkUp(); }

    double getChannel(int ch);

    void sendRxBattery(float voltage, float current, float capacity, float remaining);
    void sendAttitude(float pitch, float roll, float yaw);

private:
    HardwareSerial crsfSerial;
    AlfredoCRSF crsf;
};

#endif