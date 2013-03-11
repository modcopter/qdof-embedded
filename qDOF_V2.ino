/*
 * qDOF Hauptquelldatei. Sollte eigentlich nicht mehr als die Arduino eigenen setup und loop Funktionen enthalten,
 * alles andere in den entsprechenden Dateien.
 *
 * BY-NC-SA 2012
 */

/*
 * ---------- Header-Includes ---------
 */

// Wichtig fürs System oder Arduino-Libraries
#include <I2C.h>							// I²C-Treiber
#include <TimerOne.h>						// sollte klar sein...

#include <messages.pb.h>
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>


// eigene Header
#include "pid.h"							// PID-Code
#include "MPU6050.h"						// MPU6050 Treiber
#include "qMotorArray.h"					// Motor-Klasse
#include "crc.h"

/*
 * ---------- Ende der Includes ---------
 */

#define YAW     2
#define ROLL    1
#define PITCH   0

#define E_INVALID_PACKET    1
#define E_INSUFF_LEN        2
#define E_OK                0

unsigned long attitudeTimer = 0;

SPid ratePID[3];
SPid anglePID[2];

qMotorArray motors;
MPU6050 mpu;

Attitude currentAttitude;
Attitude setAttitude;

void setup() {
	initHardware();
	initPIDs();
	//
	attitudeTimer = millis();
	//
	motors.startupTest();
}

void loop() {
  	mpu.readAll(currentAttitude);
  	//
	if ((millis() - attitudeTimer) > 100) {
		sendAttitude();
		//
		attitudeTimer = millis();
	}
	//
	if ((abs(currentAttitude.gx) > 500) || (abs(currentAttitude.gy) > 500)) {
		motors.stop();
		return;
	}
	//
	if (motors.getSchub() < 10) {
		motors.stop();
		return;
	}
	//
	motors.restart();
	//
	UpdatePID(&anglePID[PITCH], currentAttitude.pitch);
	ratePID[PITCH].setpoint = anglePID[PITCH].output;
	UpdatePID(&ratePID[PITCH], currentAttitude.gx);

	UpdatePID(&anglePID[ROLL], currentAttitude.roll);
	ratePID[ROLL].setpoint = anglePID[ROLL].output;
	UpdatePID(&ratePID[ROLL], currentAttitude.gy);

	UpdatePID(&ratePID[YAW], currentAttitude.yaw);
	//
	motors.setAttitude(ratePID[ROLL].output,
                           ratePID[PITCH].output,
                           ratePID[YAW].output);
}

void serialEvent() {
	if(Serial.available()) {
		int packet_size = Serial.read();
		//
		uint8_t in_buffer[256];
		int bytes_read = Serial.readBytes((char*)in_buffer, packet_size);
		if (bytes_read != packet_size)
			return;
		//
		pb_istream_t istream = pb_istream_from_buffer(in_buffer, bytes_read);
		if (!pb_decode(&istream, Attitude_fields, &setAttitude))
			return;
		//
		anglePID[ROLL].setpoint = setAttitude.roll;
		anglePID[PITCH].setpoint = setAttitude.pitch;
		ratePID[YAW].setpoint = setAttitude.yaw;
		motors.setSchub(setAttitude.throttle);
	}
}

void sendAttitude() {
	uint8_t buffer[256];
	pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	pb_encode(&ostream, Attitude_fields, &currentAttitude);
	//
	Serial.write(ostream.bytes_written);
	Serial.write(buffer, ostream.bytes_written);
}
