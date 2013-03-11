#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>


typedef union __bitconverter {
	int16_t signedIntVal;
	uint16_t intVal;
	struct {
		uint8_t lByte;
		uint8_t hByte;
	} bytes;
} bitConverter_t;

/*
typedef struct __evnMess {
	float gx;			// X-Winkelgeschw. �/s
	float gy;			// Y-Winkelgeschw. �/s
	float roll;			// Roll-Winkel �
	float pitch;		// Pitch-Winkel �
	float yaw;			// Yaw-Geschwindigkeit �/s
	double temp;		// Temperatur �C
} envMess_t;*/

#endif
