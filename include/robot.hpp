#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "motor.hpp"
#include "tof.hpp"
#include "ir.hpp"
#include "encoder.hpp"
#include "servo.hpp"
#include "imu.hpp"

//#include <BluetoothSerial.h>

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


class Robot {
    public:
        Motor motorRight; 
        Motor motorLeft;

        // TOF Sensor Objects (MUST be constructed Robot.hpp, OTHERWISE ESP32 CRASHES)
        TOF tofLeftFront =    TOF(1, TOF_START_ADDR + 1, TOF3_SHT_PIN);
        TOF tofRightFront =   TOF(2, TOF_START_ADDR + 2, TOF4_SHT_PIN);
        TOF tofLeft      =    TOF(3, TOF_START_ADDR + 3, TOF5_SHT_PIN);
        TOF tofRight =        TOF(4, TOF_START_ADDR + 4, TOF6_SHT_PIN);

        IR irLeft;
        IR irRight;
        
        uint16_t turnTime;
        uint8_t turnSpeed;
        uint8_t driveSpeed;
        uint8_t maxDriveSpeed;

        uint16_t wallDistance;
        uint8_t cellWidth; //in mm
        uint8_t tofTurnError;

        //PID
        int prevError;
        // float prevTime;

        // Bluetooth Serial
        //BluetoothSerial btSerial;

        Robot();
        void setupRobot();

        // Necessary MMS methods ////////////////
        bool wallFront();
        bool wallRight();
        bool wallLeft();

        void moveForward(int distance = 1);
        void turnRight(bool disableTurnErrorCorrection = false);
        void turnLeft(bool disableTurnErrorCorrection = false);
        void turnGyroLeft(float degrees = 0.0);
        void turnGyroRight(float degrees = 0.0);
        ////////////////////////////////////////////

        void driveTillObstacle();
        bool checkForStartSignal();
        bool checkForTurnSignal();
        void correctTurnError();
        void correctSteeringError();
};

#endif