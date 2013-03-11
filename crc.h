#ifndef CRC16_H
#define CRC16_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

unsigned int calc_crc16(unsigned char *buf, unsigned short len);

#ifdef __cplusplus
}
#endif

#endif
