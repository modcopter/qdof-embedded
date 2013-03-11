// qMotorArray.h

#ifndef _QMOTORARRAY_h
#define _QMOTORARRAY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <I2C.h>

#define d(v)	Serial.print(#v); Serial.print(": "); Serial.print(v); Serial.print("  ");
#define dL(l, v)	Serial.print(l); Serial.print(": "); Serial.print(v); Serial.print("  ");


class qMotorArray {
public:
	qMotorArray() : emergencyStop(false), _schub(0), _rollOffs(0), _pitchOffs(0), _yawOffs(0) {}
	// Schub
	void setSchub(uint8_t schub);
	uint16_t getSchub();
	// Roll
	void setRollOffs(int8_t roll);
	int16_t getRollOffs();
	// Pitch
	void setPitchOffs(int8_t pitch);
	int16_t getPitchOffs();
	// Yaw
	void setYawOffs(int8_t yaw);
	int16_t getYawOffs();
	// Alles
	void setAttitude(int8_t roll, int8_t pitch, int8_t yaw);
	// Motortest
	void startupTest();
	// Not-Aus
	void stop();
	bool stopped();
	void restart();

private:
	void updateSpeeds();
	//
	bool emergencyStop;
	//
	uint8_t _schub;
	int8_t _rollOffs, _pitchOffs, _yawOffs;
};

#endif

