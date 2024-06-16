#ifndef IMU_HPP
#define IMU_HPP

#include <cstdint>
#include <Arduino.h>
#include <Wire.h>


// store the most recent sensor reading
static int16_t GyroZ;

// calculated "Turn Rate" of yaw axis in degrees per second
static float rateYaw;

// calculated angle of the mouse (in degrees) - values higher than 0: Left Turn / Lower: Right Turn
static float angleYaw;

// offset Value that is calculated in setup
static float rateCalibrationYaw;

// variables for time management, used to calculate the angle
static unsigned long int currentTime, previousTime;
static double deltaTime;

static bool isInitialized;



/**
 * @brief   Initializes and calibrates the IMU (just Gyroscope) 
 *          Requires Library Wire.h to be included (done via Adafruit Library)
 * @param   CalSamples  Number of Samples in Calibration Loop 
 *                      (circa Time it takes in millis)
*/
void initIMU(uint16_t CalSamples);


/**
 * @brief   Get new Gyro Data, put it in the rateYaw variable (°/s)
*/
void readRawGyro();


/**
 * @brief   Requires multiple, fastly repeating calls of readRawGyro()
 *          Calculates Angle Value (angleYaw variable)
*/
void calcGyro();


/**
 * @brief   Reset the Angle Value (°) to zero
*/
void setZeroAngle();


/**
 * @brief Returns true if angle is greater than degrees  
*/
bool greaterThan(float degrees);

/**
 * @brief Returns true if angle is smaller than degrees
*/
bool smallerThan(float degrees);

float getYawRate();

float getYawAngle();

#endif