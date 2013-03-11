/*
 *    In dieser Datei finden sich Funktionen zum Initialisieren der Hardware des Kopters.
 */

/**
 * @brief Initialisiert die Hardware
 *
 * 1. I2c als Master starten
 * 2. MPU initialisieren
 * 3. Kommunikation
 * 4. Debug-Pin
 * 5. Timer
 */
void initHardware() {
	Serial.begin(19200);
	//
	pinMode(13, OUTPUT);
	//
	I2c.begin();
	I2c.timeOut(100);
	I2c.pullup(true);
	//
	mpu.init();
	//
	Serial.println("Kalibiere Gyroskop....");
	mpu.calibrate();
	//
	Serial.print("Gyrosokop:   X: ");
	Serial.print(mpu.gxCorrection);
	Serial.print(" Y: ");
	Serial.print(mpu.gyCorrection);
	Serial.print(" Z: ");
	Serial.println(mpu.gzCorrection);
}

/**
 * @brief Initialisiert die PID-Regler mit ihren Standard-Werten
 * 
 * TODO: Aus EEPROM laden
 */
void initPIDs() {
	anglePID[PITCH].iMax = 100000;
	anglePID[PITCH].iMin = -100000;
	anglePID[ROLL].iMax = 100000;
	anglePID[ROLL].iMin = -100000;

	anglePID[PITCH].pGain = -2.65;
	anglePID[PITCH].iGain = -0.0075;
	ratePID[PITCH].pGain = -0.265;

	anglePID[ROLL].pGain = 2.65;
	anglePID[ROLL].iGain = 0.0075;
	ratePID[ROLL].pGain = 0.265;

	ratePID[YAW].pGain = -0.6;
}

