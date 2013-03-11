#include "MPU6050.h"

void MPU6050::init() {
  Serial.println(write_reg(MPU6050_PWR_MGMT_1, 0));
  Serial.println(write_reg(MPU6050_CONFIG, 0x06));
}

int MPU6050::testConnection() {
  uint8_t c;
  return read (MPU6050_WHO_AM_I, &c, 1);
}

#define d(v)			Serial.print(#v); Serial.print(": "); Serial.print(v);
#define l()				Serial.println();

void MPU6050::calibrate() {
	gxCorrection = gyCorrection = gzCorrection = 0;
	//
	for (int i = 0; i < 1000; i++) {
		float gx, gy, gz;
		readGyro(gx, gy, gz);
		//
		gxCorrection += gx;
		gyCorrection += gy;
		gzCorrection += gz;
		//
		if (i % 100 == 0) {
			digitalWrite(13, !digitalRead(13));
		}
	}
}

int MPU6050::readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  int error = 0;
  error = read(MPU6050_ACCEL_XOUT_H, buf, 6);
  //
  if (error == 0) {
    bitConverter.bytes.hByte = buf[0];
    bitConverter.bytes.lByte = buf[1];
    ax = bitConverter.signedIntVal;
    //
    bitConverter.bytes.hByte = buf[2];
    bitConverter.bytes.lByte = buf[3];
    ay = bitConverter.signedIntVal;
    //
    bitConverter.bytes.hByte = buf[4];
    bitConverter.bytes.lByte = buf[5];
    az = bitConverter.signedIntVal;
  }
  //
  return error;
}

int MPU6050::readGyro(float &gx, float &gy, float &gz) {
  int error = 0;
  error = read(MPU6050_GYRO_XOUT_H, buf, 6);
  //
  if (error == 0) {
    bitConverter.bytes.hByte = buf[0];
    bitConverter.bytes.lByte = buf[1];
    gx = bitConverter.signedIntVal * GYRO_FAKTOR - this->gxCorrection;
    //
    bitConverter.bytes.hByte = buf[2];
    bitConverter.bytes.lByte = buf[3];
    gy = bitConverter.signedIntVal * GYRO_FAKTOR - this->gyCorrection;
    //
    bitConverter.bytes.hByte = buf[4];
    bitConverter.bytes.lByte = buf[5];
    gz = bitConverter.signedIntVal * GYRO_FAKTOR - this->gzCorrection;
  }
  //
  return error;
}

float MPU6050::readTemperature() {
  int error = read(MPU6050_TEMP_OUT_H, buf, 2);
  //
  if (error == 0) {
    bitConverter.bytes.hByte = buf[0];
    bitConverter.bytes.lByte = buf[1];
    //Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
    return (bitConverter.signedIntVal) / 340.0 + 36.53;
  } else {
    return -1;
  }
}

void MPU6050::readAll(Attitude &envMess) {
	int16_t ax = 0, ay = 0, az = 0;
	float gx = 0, gy = 0, gz = 0;
	//
	readAccel(ax, ay, az);
	readGyro(gx, gy, gz);
	//
	envMess.gx = gx;
	envMess.gy = gy;
	//
	envMess.yaw = gz;
	envMess.roll = (atan2(ax, az)) * RAD_TO_DEG;
	envMess.pitch = (atan2(ay, az)) * RAD_TO_DEG;
	//
	envMess.temperature = readTemperature();
}

int MPU6050::read(int start, uint8_t *buffer, int size) {
  return I2c.read(MPU6050_I2C_ADDRESS, start, size, buffer);
}

int MPU6050::write(int start, uint8_t *pData, int size) {
  return I2c.write(MPU6050_I2C_ADDRESS, start, pData, size);
}

int MPU6050::write_reg(uint8_t reg, uint8_t data) {
  return I2c.write((uint8_t)MPU6050_I2C_ADDRESS, reg, data);
}

float MPU6050::smooth(float data, float filterVal, float smoothedVal) {
	if (filterVal > 1){      // check to make sure param's are within range
		filterVal = .99;
	} else if (filterVal <= 0){
		filterVal = 0;
	}

	smoothedVal = (data * (1.0 - filterVal)) + (smoothedVal  *  filterVal);

	return smoothedVal;
}

