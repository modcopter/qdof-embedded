// 
// 
// 

#include "qMotorArray.h"

#undef MOTORS_SAVE

#define MOTOR1		(uint8_t)0x13
#define MOTOR2		(uint8_t)0x12
#define MOTOR3		(uint8_t)0x11
#define MOTOR4		(uint8_t)0x10

#define M1_FAKTOR	1.11
#define M2_FAKTOR	0.89
#define M3_FAKTOR	1.005
#define M4_FAKTOR	0.995

// Schub
void qMotorArray::setSchub(uint8_t schub) {
	_schub = schub;
	updateSpeeds();
}
uint16_t qMotorArray::getSchub() {
	return _schub;
}

// Roll
void qMotorArray::setRollOffs(int8_t roll) {
	_rollOffs = constrain(roll, -255, 255);
	updateSpeeds();
}
int16_t qMotorArray::getRollOffs() {
	return _rollOffs;
}

// Pitch
void qMotorArray::setPitchOffs(int8_t pitch) {
	_pitchOffs = constrain(pitch, -255, 255);
	updateSpeeds();
}
int16_t qMotorArray::getPitchOffs() {
	return _pitchOffs;
}

// Yaw
void qMotorArray::setYawOffs(int8_t yaw) {
	_yawOffs = constrain(yaw, -255, 255);
	updateSpeeds();
}
int16_t qMotorArray::getYawOffs() {
	return _yawOffs;
}

// Alles
void qMotorArray::setAttitude(int8_t roll, int8_t pitch, int8_t yaw) {
	_rollOffs = constrain(roll, -255, 255);
	_pitchOffs = constrain(pitch, -255, 255);
	_yawOffs = constrain(yaw, -255, 255);
	//
	updateSpeeds();
}

#define FLASH_MOTOR(a)		I2c.write(a, (uint8_t)15); delay(250); I2c.write(a, (uint8_t)0);

void qMotorArray::startupTest() {
#ifndef MOTORS_SAVE
	FLASH_MOTOR(MOTOR1);
	delay(100);
	FLASH_MOTOR(MOTOR2);
	delay(100);
	FLASH_MOTOR(MOTOR3);
	delay(100);
	FLASH_MOTOR(MOTOR4);
#endif
}

void qMotorArray::stop() {
	// Jetzt sind sie schonmal aus!
	I2c.write(MOTOR1, (uint8_t)0);
	I2c.write(MOTOR2, (uint8_t)0);
	I2c.write(MOTOR3, (uint8_t)0);
	I2c.write(MOTOR4, (uint8_t)0);
	// Flag setzen
	emergencyStop = true;
}

bool qMotorArray::stopped() {
	return emergencyStop;
}

void qMotorArray::restart() {
	emergencyStop = false;
}

void qMotorArray::updateSpeeds() {
	if (!emergencyStop) {
		uint8_t pwm1, pwm2, pwm3, pwm4;
		// PITCH
		pwm1 = constrain(M1_FAKTOR * (_schub - _pitchOffs + _yawOffs), 0, 255);
		pwm3 = constrain(M2_FAKTOR * (_schub + _pitchOffs + _yawOffs), 0, 255);
		// ROLL
		pwm4 = constrain(M3_FAKTOR * (_schub - _rollOffs - _yawOffs), 0, 255);
		pwm2 = constrain(M4_FAKTOR * (_schub + _rollOffs - _yawOffs), 0, 255);

		//s_1 = f_1 * (T - pitch + yaw)
		//s_2 = f_2 * (T + roll - yaw)
		//s_3 = f_3 * (T + pitch + yaw)
		//s_4 = f_4 * (T - roll - yaw)

		//s_1 = f_1 * (T - pitch + yaw)
		//s_3 = f_3 * (T + pitch + yaw)

		//s_2 = f_2 * (T + roll - yaw)
		//s_4 = f_4 * (T - roll - yaw)

		//s_1,3 = f_1,3 * (T -+ pitch + yaw)
		//s_2,4 = f_2,4 * (T +- roll - yaw)
#ifndef MOTORS_SAVE
		I2c.write(MOTOR1, pwm1);
		I2c.write(MOTOR2, pwm2);
		I2c.write(MOTOR3, pwm3);
		I2c.write(MOTOR4, pwm4);
#endif
	} else {
		I2c.write(MOTOR1, (uint8_t)0);
		I2c.write(MOTOR2, (uint8_t)0);
		I2c.write(MOTOR3, (uint8_t)0);
		I2c.write(MOTOR4, (uint8_t)0);
	}
}
