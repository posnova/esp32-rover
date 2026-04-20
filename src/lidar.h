#ifndef _LIDAR_H_
#define _LIDAR_H_

#include <stdint.h>
#include <HardwareSerial.h>
#include "gpio.h"

typedef struct __attribute__((packed)) {
    uint16_t distance;   // Unit: mm
    uint8_t confidence;  // signal strength
} LidarPoint;


typedef struct __attribute__((packed)) {
    uint8_t header;         // 0x54
    uint8_t ver_len;        // 0x2C
    uint16_t speed;         // degrees/sec
    uint16_t start_angle;   // 0.01 degree units
    LidarPoint points[12];
    uint16_t end_angle;     // 0.01 degree units
    uint16_t timestamp;
    uint8_t crc8;
} LidarPacket;


typedef enum {
    FIND_HEADER,
    FIND_VER_LEN,
    GATHER_DATA
} ParseState;


#define PACKET_LEN 47


class Lidar {
public:
    Lidar(): serial(LIDAR_SERIAL) {}
    void begin();
    LidarPacket* update();

private:
    bool validate_crc();

private:
    HardwareSerial serial;
    ParseState current_state = FIND_HEADER;
    uint8_t packet_idx = 0;
    uint8_t raw_packet[PACKET_LEN];
};

#endif