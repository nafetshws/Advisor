#include <cstdint>
#include "BluetoothSerial.h"
#include "../include/robot.hpp"
#include "../include/encoder.hpp"
#include "../include/imu.hpp"
#include "../include/floodfill.hpp"

Robot::Robot() {
  this->motorRight = Motor(MOTORA_IN1, MOTORA_IN2, MOTORA_PWM, MOTORA_PWM_CHANNEL);
  this->motorLeft = Motor(MOTORB_IN1, MOTORB_IN2, MOTORB_PWM, MOTORB_PWM_CHANNEL);

  // IR Sensor objects
  this->irLeft = IR (TOF1_SHT_PIN);
  this->irRight = IR (TOF2_SHT_PIN);

  this->turnTime = 270;
  this->turnSpeed = 500;
  this->driveSpeed = 600;
  this->wallDistance = 80;
  this->cellWidth = 160;
  this->tofTurnError = 10;
  this->maxDriveSpeed = 800;

  this->prevError = 0.0f;
  this->KP = 0.3f;
  this->KD = 0.35f;

}

void Robot::setupRobot() {
  // SETUP SERIAL MONITOR ///////////////////////

  // start serial monitor
  Serial.begin(115200);
  // start 
  btSerial.begin("BallE BluetoothTestInterface");
  Serial.println("\nSETUP: Serial Monitor running");

  // SETUP MOTORS ///////////////////////////////

  // Initialise the motor objects, start pwm channels, confiure pins
  motorRight.initialise();
  motorLeft.initialise();
  // Initialise the motor encoders pins and interrupts, init timer interrupt
  initEncoders(MOTORA_ENCODER1, MOTORA_ENCODER2, MOTORB_ENCODER1, MOTORB_ENCODER2);

  Serial.println("SETUP: Motor initialised");


   // SETUP TOF //////////////////////////////////
   Serial.println("SETUP: Try to connect to TOF sensors...");

  // init all the tof sensors

  // initTofSensors(tofLeftFront, tofRightFront, tofLeft, tofRight);
  initTofSensors(tofLeftFront, tofRightFront);;
  initTofSensors(tofLeft, tofRight);

   Serial.println("SETUP: TOF Sensors initialised");

  // SETUP DIP switches /////////////////////////
  pinMode(DIP_SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(DIP_SWITCH_PIN_2, INPUT_PULLUP);


  // SETUP IMU //////////////////////////////////
  initIMU(2000);

  // SETUP END //////////////////////////////////
  Serial.println("SETUP: Setup Done");
}

void Robot::driveTillObstacle() {
  motorRight.turnForward(driveSpeed);
  motorLeft.turnForward(driveSpeed);
 
  while (!irLeft.isTriggered() && !irRight.isTriggered()) {
    this->correctSteeringError();

    delay(5);
  }

  motorRight.stopMotor();
  motorLeft.stopMotor();
}

void Robot::turnLeftWithEncoders() {
  uint32_t startEncLeftValue = getEncLeft();
  uint32_t startEncRightValue = getEncRight();

  motorLeft.turnForward(turnSpeed);
  motorRight.turnBackward(turnSpeed);

  uint32_t currentEncLeft = startEncLeftValue;
  uint32_t currentEncRight = startEncRightValue;

  do {
    if (startEncLeftValue + TURN_ENC_TICKS >= currentEncLeft) {
      motorLeft.stopMotor();
    }
    if (startEncRightValue + TURN_ENC_TICKS >= currentEncRight) {
      motorRight.stopMotor();
    }
  } while (startEncLeftValue  + TURN_ENC_TICKS < currentEncLeft ||
           startEncRightValue + TURN_ENC_TICKS < currentEncRight);
}

void Robot::turnRightWithEncoders() {
  uint32_t startEncLeftValue = getEncLeft();
  uint32_t startEncRightValue = getEncRight();

  motorLeft.turnBackward(turnSpeed);
  motorRight.turnForward(turnSpeed);

  uint32_t currentEncLeft = startEncLeftValue;
  uint32_t currentEncRight = startEncRightValue;

  do {
    if (startEncLeftValue + TURN_ENC_TICKS >= currentEncLeft) {
      motorLeft.stopMotor();
    }
    if (startEncRightValue + TURN_ENC_TICKS >= currentEncRight) {
      motorRight.stopMotor();
    }
  } while (startEncLeftValue  + TURN_ENC_TICKS < currentEncLeft ||
           startEncRightValue + TURN_ENC_TICKS < currentEncRight);
}

void Robot::turnRight(bool disableTurnErrorCorrection) {
  long startTime = millis();

  motorRight.turnBackward(turnSpeed);
  motorLeft.turnForward(turnSpeed);

  while (millis() - startTime < turnTime) {
    delay(3);
  }

  motorRight.stopMotor();
  motorLeft.stopMotor();

  if (!disableTurnErrorCorrection) this->correctTurnError();
}

void Robot::turnLeft(bool disableTurnErrorCorrection) {
  long startTime = millis();
  
  motorRight.turnForward(turnSpeed);
  motorLeft.turnBackward(turnSpeed);

  while (millis() - startTime < turnTime) {
    delay(3);
  }

  motorLeft.stopMotor();
  motorRight.stopMotor();

  if (!disableTurnErrorCorrection) this->correctTurnError();
}

void Robot::turnRightWithGyro(float degrees) {
  setZeroAngle();         // Resets Angle

  while(smallerThan(degrees)) {
    readRawGyro();        // Get new Data
    calcGyro();           // Calculate new Angle
  
    motorRight.turnBackward(turnSpeed); // Turn Mouse
    motorLeft.turnForward(turnSpeed);
  
    delay(2);
  }

  btSerial.printf("Turned %f degrees.\n", getYawAngle());

  motorLeft.stopMotor();
  motorRight.stopMotor();

  for (int i = 0; i < 50; i++) {
    readRawGyro();        // Get new Data
    calcGyro();           // Calculate new Angle

    btSerial.printf("Turned final %f degrees.\n", getYawAngle());
    delay(1);
  }


  resetLeftEncoder();
  resetRightEncoder();
  delay(1000);
}

void Robot::turnLeftWithGyro(float degrees) {
  setZeroAngle();         // Resets Angle

  degrees *= -1.0;
  
  while(greaterThan(degrees)) { 
    readRawGyro();        // Get new Data
    calcGyro();           // Calculate new Angle

    motorRight.turnForward(turnSpeed); // Turn Mouse
    motorLeft.turnBackward(turnSpeed);
  
    delay(2);
  }

  motorLeft.stopMotor();
  motorRight.stopMotor();
  resetLeftEncoder();
  resetRightEncoder();
  delay(1000);
}

/*************Deprecated**********+***/
void Robot::correctTurnError() {
  uint16_t currentDistanceLeft;
  uint16_t currentDistanceRight;
  int distanceDifference;

  do {
    currentDistanceLeft = tofLeftFront.getDist();
    currentDistanceRight = tofRightFront.getDist();
    distanceDifference = currentDistanceLeft - currentDistanceRight;

    if (distanceDifference < 0) {
      // Turn right
      uint16_t turnTimeTmp = this->turnTime;
      // rotation time is proportional to the distance difference between the tof sensors
      this->turnTime = abs(distanceDifference) / 4;
      // Calls turnRight(), but disables error correction to avoid infinite recursion
      this->turnRight(true);
      this->turnTime = turnTimeTmp;
    } else {
      // Turn left
      uint16_t turnTimeTmp = this->turnTime;
      // rotation time is proportional to the distance difference between the tof sensors
      this->turnTime = abs(distanceDifference) / 4;
      // Calls turnLeft(), but disables error correction to avoid infinite recursion
      this->turnLeft(true);
      this->turnTime = turnTimeTmp;
    }

  } while (abs(distanceDifference) >= tofTurnError);
}
/*********************************+***/

bool Robot::checkForTurnSignal() {
  int switch1 = digitalRead(DIP_SWITCH_PIN_1) == 0 ? 1 : 0;
  int switch2 = digitalRead(DIP_SWITCH_PIN_2) == 0 ? 1 : 0;


  return (switch1 == LOW && switch2 == HIGH); 
}

bool Robot::checkForStartSignal() {
  int switch1 = digitalRead(DIP_SWITCH_PIN_1) == 0 ? 1 : 0;
  int switch2 = digitalRead(DIP_SWITCH_PIN_2) == 0 ? 1 : 0;

  // btSerial.printf("DIP1: %d\t DIP2: %d\n", switch1, switch2);

  return switch1 == HIGH && switch2 == HIGH; 
}

bool Robot::wallFront() {
  this->btSerial.println("Front Wall");
  bool hasWall;
  bool hasChanged = false;
  while (!hasChanged) {
    if (this->btSerial.available()) {
      std::string s(btSerial.readStringUntil('\n').c_str());
      hasWall = stoi(s);
      hasChanged = true;
    }
  }

  // hasWall = tofLeftFront.getDist() < this->wallDistance || tofRightFront.getDist() < this->wallDistance;
  // hasWall = tofRightFront.getDist() < this->wallDistance;
  // this->btSerial.printf("Front wall measured: %d\n", hasWall);
  // return hasWall;
  // bool hasWall = irLeft.isTriggered() && irRight.isTriggered();
  this->btSerial.printf("Front wall measured: %d\n", hasWall);
  return hasWall;
}

bool Robot::wallRight() {
   this->btSerial.println("Right Wall");
  bool hasWall;
  bool hasChanged = false;
  while (!hasChanged) {
    if (this->btSerial.available()) {
      std::string s(btSerial.readStringUntil('\n').c_str());
      hasWall = stoi(s);
      hasChanged = true;
    }
  }

  // hasWall = tofRight.getDist() < this->wallDistance;
  this->btSerial.printf("Right wall measured: %d\n", hasWall);
  return hasWall;
}

bool Robot::wallLeft() {
this->btSerial.println("Left Wall");
bool hasWall;
bool hasChanged = false;
  while (!hasChanged) {
    if (this->btSerial.available()) {
      std::string s(btSerial.readStringUntil('\n').c_str());
      hasWall = stoi(s);
      hasChanged = true;
    }
  }

  // hasWall = tofLeft.getDist() < this->wallDistance;
  this->btSerial.printf("Left wall measured: %d\n", hasWall);
  return hasWall;
}

/*************Deprecated**********+***/
void Robot::moveForwardUsingToF(int distance) {
  uint16_t startDistanceLeft = tofLeftFront.getDist();
  uint16_t startDistanceRight = tofRightFront.getDist();
  uint16_t startDistance = (startDistanceLeft + startDistanceRight) / 2;

  motorRight.turnForward(this->driveSpeed);
  motorLeft.turnForward(this->driveSpeed);

  uint16_t currentDistanceLeft;
  uint16_t currentDistanceRight;
  uint16_t currentDistanceToWall;

  do {
    // Insert error correction /////
    this->correctSteeringError();
    ////////////////////////////////
    currentDistanceLeft = tofLeftFront.getDist();
    currentDistanceRight = tofRightFront.getDist();
    currentDistanceToWall = (currentDistanceLeft + currentDistanceRight) / 2;
  } while (startDistance - (this->cellWidth * distance) <= currentDistanceToWall); 

  motorLeft.stopMotor();
  motorRight.stopMotor();
}
/*********************************+***/


void Robot::moveForwardUsingEncoders(int distance) {
  uint32_t startValueEncLeft = getEncLeft();
  uint32_t startValueEncRight = getEncRight();

  // TODO: update value for circumference
  float wheelRotationsNeeded = (distance * 15) / WHEEL_CIRCUMFERENCE;
  // Encoder increments 3 times per motor revolution * 30 (Gear ratio) = 90
  float motorRotationsNeeded = wheelRotationsNeeded * 90;

  // motorLeft.turnForward(this->driveSpeed);
  // motorRight.turnForward(this->driveSpeed + 25);

  uint16_t speedDelta = this->driveSpeed / 5;

  // Accelerate ofer a time frame of 100 ms to the desired speed
  for (int i = 0; i < 5; i++) {
    motorLeft.turnForward(motorLeft.getSpeed()   + speedDelta);
    motorRight.turnForward(motorRight.getSpeed() + speedDelta);
    delay(20);
  }

  // Move forward until distance is covered, while correcting the steering error
  do {
      this->correctSteeringError();
      delay(5);
  } while (getEncLeft()  - startValueEncLeft  < motorRotationsNeeded && 
           getEncRight() - startValueEncRight < motorRotationsNeeded);

  // Break over a time frame of 100 ms 
/*   for (int i = 0; i < 5; i++) {
    motorLeft.turnForward(motorLeft.getSpeed()  - speedDelta);
    motorRight.turnForward(motorLeft.getSpeed() - speedDelta);
    delay(20);
  } */

  motorLeft.stopMotor();
  motorRight.stopMotor();

  resetLeftEncoder();
  resetRightEncoder();

  delay(1000);
}

void Robot::correctSteeringError() {
  // Simple PD algorithm that uses encoders and a gyro sensor to correct errors when driving straight

  // Caluclate error reported by the encoders for the proportional term
  int error = getEncLeft() - getEncRight();

  // Calculate Proportional Term
  float proportional = this->KP * error;
  // Update gyro values
  readRawGyro();
  // Calculate Derivative Term - YawRate is given in degree/s (clockwise rotation: > 0, counter-clockwise < 0)
  float derivative = this->KD * getYawRate(); 

  float pidTerm = proportional + derivative;

  // Calculate adjusted motor speeds
  uint16_t leftMotorSpeedPid = this->motorLeft.getSpeed() - (int) pidTerm;
  uint16_t rightMotorSpeedPid = this->motorRight.getSpeed() + (int) pidTerm;

  // Debug info
  btSerial.printf("e: %d  EncL: %d  EncR: %d  proportional: %f  derivative: %f  PidTerm: %f\n", error, getEncLeft(), getEncRight(), proportional, derivative, pidTerm);
  Serial.printf  ("e: %d  EncL: %d  EncR: %d  proportional: %f  derivative: %f  PidTerm: %f\n", error, getEncLeft(), getEncRight(), proportional, derivative, pidTerm);

  // Prevent PD from going too fast
  if (leftMotorSpeedPid > this->maxDriveSpeed) { 
    leftMotorSpeedPid = this->maxDriveSpeed;
  }
  if (rightMotorSpeedPid > this->maxDriveSpeed)
  {
    rightMotorSpeedPid = this->maxDriveSpeed;
  }

  // Sets each motor to the pd adjusted speed ////////
  this->motorLeft.turnForward(leftMotorSpeedPid);
  this->motorRight.turnForward(rightMotorSpeedPid);
  ////////////////////////////////////////////////////
}

void Robot::startFloodfill() {
  Maze::initMaze();
  Maze::attachRobot(this);
  floodfill(*Maze::startCell);
  Maze::dettachRobot();
}