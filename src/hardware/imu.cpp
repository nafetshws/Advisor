#include "../../include/imu.hpp"


void initIMU(uint16_t CalSamples) {
  isInitialized = false;

  Wire.begin();
  delay(200);

  Serial.println("SETUP: Start Gyro Calibration");

  Wire.beginTransmission(0x68);
    Wire.write(0x6B);               // Turn on the MPU6050
    Wire.write(0x00);
  Wire.endTransmission();

  // int16_t i;
  // for (i = 0; i < CalSamples; i++){  // Calibration -> Mouse needs to be still
  //   readRawGyro();
  //   rateCalibrationYaw += rateYaw;
  //   delay(1); 
  // }

  // rateCalibrationYaw /= CalSamples; // // Calculate Calibration Values

  rateCalibrationYaw = 0.3;

  Serial.println("Gyro Calibrated");
  Serial.println(rateCalibrationYaw);

  isInitialized = true;

}

void readRawGyro() {
  
    Wire.beginTransmission(0x68); 
        Wire.write(0x1A);           // Turn on LOW PASS Filter to reduce noise
        Wire.write(0x05);
    Wire.endTransmission();


    Wire.beginTransmission(0x68);
        Wire.write(0x1B);            // Set Sensitivity Scale: 65.5 LSB
        Wire.write(0x08);            // -> 1°/s is 65.5 LSB
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
        Wire.write(0x47);            // Get Data from GYRO_ZOUT_H register
        Wire.endTransmission();
    Wire.requestFrom(0x68, 2);       // Request 2 bytes for the Z-axis

    GyroZ = Wire.read() << 8 | Wire.read();

    rateYaw = (float) GyroZ / 65.5; // Conversion of the values

    if (isInitialized) rateYaw -= rateCalibrationYaw; // Offset from the calibration
}

void setZeroAngle() {
  angleYaw = 0; 
}

void calcGyro() {
  currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0; // Get delta between last measurements,
                                                     //Convert milliseconds to seconds
  previousTime = currentTime;
  if(rateYaw > 0.3 || rateYaw < -0.3) {  // Here a simple filter is used (may need testing)
    angleYaw += rateYaw * deltaTime;    // Integrate to get the angle
  }

  // Serial.printf("Current time: %ld \n", currentTime);;
  // Serial.print(rateYaw);
  // Serial.print("\t");
  // Serial.println(angleYaw);

}

bool greaterThan (float degrees) {
    return angleYaw > degrees;
}

bool smallerThan(float degrees) {
    return angleYaw < degrees;
}

float getYawRate() {
  return rateYaw;
}

float getYawAngle() {
  return angleYaw;
}
