#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "motor.hpp"
#include "tof.hpp"
#include "ir.hpp"
#include "encoder.hpp"
#include "servo.hpp"
#include "imu.hpp"

#include <BluetoothSerial.h>
#include <HardwareSerial.h>

// ESP32 PIN OUT ////////////////////////////////

// PINS MOTOR A
#define MOTORA_IN1  0
#define MOTORA_IN2  2
#define MOTORA_PWM  15
#define MOTORA_ENCODER1 36
#define MOTORA_ENCODER2 39
// PINS MOTOR B
#define MOTORB_IN1  4
#define MOTORB_IN2  16
#define MOTORB_PWM  17
#define MOTORB_ENCODER1 35
#define MOTORB_ENCODER2 34

// PINS TOF SENSORS
#define TOF1_SHT_PIN 32
#define TOF2_SHT_PIN 33
#define TOF3_SHT_PIN 25
#define TOF4_SHT_PIN 26
#define TOF5_SHT_PIN 27
#define TOF6_SHT_PIN 14

// DIP Switches
#define DIP_SWITCH_PIN_1 13
#define DIP_SWITCH_PIN_2 12

// SERVO Motor
#define SERVO_PIN 23


// Error bounds
#define MIN_ERROR_THRESHOLD 10
#define MAX_ERROR_THRESHOLD 100

// TODO: Update correct value
#define WHEEL_CIRCUMFERENCE (12.56f)

#define TURN_ENC_TICKS 51


class Robot {
    public:
        Motor motorRight; 
        Motor motorLeft;

        // TOF Sensor Objects (MUST be constructed Robot.hpp, OTHERWISE ESP32 CRASHES)
        TOF_6180 tofLeftFront =    TOF_6180(1, TOF_START_ADDR + 1, TOF3_SHT_PIN);
        TOF_6180 tofRightFront =   TOF_6180(2, TOF_START_ADDR + 2, TOF4_SHT_PIN);
        TOF_6180 tofLeft  =   TOF_6180(3, TOF_START_ADDR + 3, TOF5_SHT_PIN);
        TOF_6180 tofRight =   TOF_6180(4, TOF_START_ADDR + 4, TOF6_SHT_PIN);

        IR irLeft;
        IR irRight;
        
        uint16_t turnTime;
        uint16_t turnSpeed;
        uint16_t driveSpeed;
        uint16_t correctionSpeed;
        uint16_t maxDriveSpeed;

        uint16_t wallDistance;
        uint8_t cellWidth; //in mm
        uint8_t tofTurnError;

        //PID
        int prevError;
        float KP;
        float KD;

        // Bluetooth Serial
        BluetoothSerial btSerial;

        HardwareSerial SerialMatrix;

        // dt
        unsigned long prevTime;

        // Deprecated - used for turning
        float rightBrake;
        float leftBrake;

        // Error correction
        uint16_t counterSinceLastCorrection;

        Robot();
        void setupRobot();

        // Necessary MMS methods ////////////////
        bool wallFront();
        bool wallRight();
        bool wallLeft();
        void moveForwardUsingEncoders(int distance = 1);
        void turnLeft(float degrees = 90);
        void turnRight(float degress = 90);
        ////////////////////////////////////////////

        // Algorithms
        void startFloodfill();
        void ballPickUp();
        void driveTillObstacle();

        // Helper functions
        int16_t calcAverageDifference(TOF_6180 &tof1, TOF_6180 &tof2, int samples = 3);
        void smallAdjustmentGyro(float degrees, bool turnLeft);
        void turnLeftWithGyro(float degrees = 90);
        void turnRightWithGyro(float degrees = 90);
        void alignRobot(TOF_6180 &tof1, TOF_6180 &tof2);
        void driveToMiddle(TOF_6180 &l1, TOF_6180 &l2, TOF_6180 &r1, TOF_6180 &r2);
        uint16_t calcAverageDistance(TOF_6180 &tof, int samples);

        // Error correction
        void correctSteeringError(); //PD - Control
        void correctTurnError();
        void correctWithFrontWall(); 
        void cellCorrectionWithToF(TOF_6180 &l1, TOF_6180 &r1, TOF_6180 &r2); 
        void correctFrontDistance();
        void calibrateToFOffsets();

        void correctRobot(boolean isWallFront, boolean isWallLeft, boolean isWallRight);

        //DIP switch control
        bool checkForStartSignal();
        bool checkForTurnSignal();

        // Deprecated methods ////////////////////
        void moveForwardUsingToF(int distance = 1);
        void turnLeftWithGyroErrorCorrection(float degrees = 90);
        void turnRightWithGyroErrorCorrection(float degrees = 90);
        float leftGyroHelper(float degrees = 0.0);
        float rightGyroHelper(float degrees = 0.0);
        void turnRightWithEncoders();
        void turnLeftWithEncoders();
        void turnRightSimple(bool disableTurnErrorCorrection = false);
        void turnLeftSimple(bool disableTurnErrorCorrection = false);
        
        void sendHeadChar(char i);
        //////////////////////////////////////////
};


#endif