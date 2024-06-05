#include <cstdint>
#include "BluetoothSerial.h"
#include "../include/robot.hpp"
#include "../include/encoder.hpp"
#include "../include/imu.hpp"

Robot::Robot() {
  this->motorRight = Motor(MOTORA_IN1, MOTORA_IN2, MOTORA_PWM, MOTORA_PWM_CHANNEL);
  this->motorLeft = Motor(MOTORB_IN1, MOTORB_IN2, MOTORB_PWM, MOTORB_PWM_CHANNEL);

  // IR Sensor objects
  this->irLeft = IR (TOF1_SHT_PIN);
  this->irRight = IR (TOF2_SHT_PIN);

  this->turnTime = 270;
  this->turnSpeed = 500;
  this->driveSpeed = 600;
  this->wallDistance = 120;
  this->cellWidth = 160;
  this->tofTurnError = 10;
  this->maxDriveSpeed = 800;

  this->prevError = 0.0f;
  this->KP = 0.3f;
  this->KD = 0.4f;

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
  #include "../include/imu.hpp"
  motorLeft.initialise();
  // Initialise the motor encoders pins and interrupts, init timer interrupt
  initEncoders(MOTORA_ENCODER1, MOTORA_ENCODER2, MOTORB_ENCODER1, MOTORB_ENCODER2);

  Serial.println("SETUP: Motor initialised");


  // // SETUP TOF //////////////////////////////////
  // Serial.println("SETUP: Try to connect to TOF sensors...");

  // // init all the tof sensors

  // initTofSensors(tofLeftFront, tofRightFront, tofLeft, tofRight);
  // // initTofSensors(tofLeftFront, tofRightFront);;
  // // initTofSensors(tofLeft, tofRight);

  // Serial.println("SETUP: TOF Sensors initialised");

  // // SETUP DIP switches /////////////////////////
  pinMode(DIP_SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(DIP_SWITCH_PIN_2, INPUT_PULLUP);


  // SETUP IMU //////////////////////////////////
  initIMU(2000);

  // SETUP END //////////////////////////////////
  Serial.println("SETUP: Setup Done");
}

void Robot::driveTillObstacle() {
  uint16_t max_distance = 60;

  // uint16_t topLeftDistance  = this->tofLeftFront.getDist();
  // uint16_t topRightDistance = this->tofRightFront.getDist();
  // uint16_t irLeftDistance   = this->irLeft.isTriggered();
  // uint16_t irRightDistance  = this->irRight.isTriggered();

  motorRight.turnForward(driveSpeed);
  motorLeft.turnForward(driveSpeed);

  this->prevTime = millis();
  
  // Serial.printf("TOF top left: %d\t TOF top rigth: %d\n", topLeftDistance, topRightDistance);

  // while (topLeftDistance > max_distance && topRightDistance > max_distance) {
  while (!irLeft.isTriggered() && !irRight.isTriggered()) {
    // topLeftDistance = tofLeftFront.getDist();
    // topRightDistance = tofRightFront.getDist();
    // irLeftDistance = irLeft.isTriggered();
    // irRightDistance = irRight.isTriggered();

    this->correctSteeringError();

    // Serial.printf("TOF top left: %d\t TOF top rigth: %d\n", topLeftDistance, topRightDistance);

    // uint32_t leftEncoder = getEncLeft();
    // uint32_t rightEncoder = getEncRight();

    // Serial.printf("Left enc: %d\tRight enc: %d\tDifference: %d\n", leftEncoder, rightEncoder, leftEncoder - rightEncoder);
    // btSerial.printf("Left enc: %d\tRight enc: %d\tDifference: %d\n", leftEncoder, rightEncoder, leftEncoder - rightEncoder);
    delay(5);
  }

  motorRight.stopMotor();
  motorLeft.stopMotor();

  btSerial.printf("Checking if the robot is turned correctly.\n");

  delay(1000);

  // uint16_t tofLeftFrontDistance = tofLeftFront.getDist();
  // uint16_t tofRightFrontDistance = tofRightFront.getDist();
  // uint8_t maxDistance = 45;

  // bool leftStopped = false;
  // bool rightStopped = false;

  // while (tofLeftFrontDistance > maxDistance || tofRightFrontDistance > maxDistance) {
  //   if (tofLeftFrontDistance > maxDistance && !leftStopped) {
  //     motorLeft.turnForward(50);
  //   } else {
  //     motorLeft.stopMotor();
  //     leftStopped = true;
  //   }

  //   if (tofRightFrontDistance > maxDistance && !rightStopped) {
  //     motorRight.turnForward(50);
  //   } else {
  //     motorRight.stopMotor();
  //     rightStopped = true;
  //   }

  //   if (leftStopped && rightStopped) break;
  // }

  // while (tofLeftFront.getDist() > 45) {
  //   motorLeft.turnForward(50);
  // }

  // motorLeft.stopMotor();

  // while (tofRightFront.getDist() > 45) {
  //   motorRight.turnForward(50);
  // }

  // motorRight.stopMotor();

  // Correct the robot, before it's about to turn
  // if (!irLeft.isTriggered()) {
  //   // Only use left motor until both ir sensors are triggered
  //   btSerial.printf("Correcting left\n");
  //   while (!irLeft.isTriggered()) {
  //     motorLeft.turnForward(50);
  //   } 
  //   motorLeft.stopMotor();
  // } else if (!irRight.isTriggered()) {
  //   // Only use rigth motor until both ir sensors are triggered
  //   btSerial.printf("Correcting left\n");
  //   while (!irRight.isTriggered()) {
  //     motorRight.turnForward(50);
  //   } 
  //   motorRight.stopMotor();
  // } else {
  //   btSerial.printf("Left: %d, Right: %d\n", irLeft.isTriggered(), irRight.isTriggered());
  // }

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

void Robot::turnGyroRight(float degrees) {
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
}

void Robot::turnGyroLeft (float degrees) {
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
}

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
  return tofLeftFront.getDist() < this->wallDistance || tofRightFront.getDist() < this->wallDistance;
}

bool Robot::wallRight() {
  return tofRight.getDist() < this->wallDistance;
}

bool Robot::wallLeft() {
  return tofLeft.getDist() < this->wallDistance;
}

void Robot::moveForward(int distance) {
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

void Robot::moveForwardUsingEncoders(int distance) {
  uint32_t startValueEncLeft = getEncLeft();
  uint32_t startValueEncRight = getEncRight();

  // TODO: update value for circumference
  float wheelRotationsNeeded = (distance * 17) / WHEEL_CIRCUMFERENCE;
  // Encoder increments 3 times per motor revolution * 30 (Gear ratio) = 90
  float motorRotationsNeeded = wheelRotationsNeeded * 90;

  // motorLeft.turnForward(this->driveSpeed);
  // motorRight.turnForward(this->driveSpeed + 25);

  uint16_t speedDelta = this->driveSpeed / 5;
  uint16_t speedDeltaRight = speedDelta + 5;

  for (int i = 0; i < 5; i++) {
    motorLeft.turnForward(motorLeft.getSpeed() + speedDelta);
    motorRight.turnForward(motorRight.getSpeed() + speedDeltaRight);
    delay(100);
  }

  do {
      this->correctSteeringError();
  } while (getEncLeft()  - startValueEncLeft  < motorRotationsNeeded && 
           getEncRight() - startValueEncRight < motorRotationsNeeded);

  for (int i = 0; i < 5; i++) {
    motorLeft.turnForward(motorLeft.getSpeed() - speedDelta);
    motorRight.turnForward(motorLeft.getSpeed() - speedDeltaRight);
    delay(100);
  }

  motorLeft.stopMotor();
  motorRight.stopMotor();
}

void Robot::correctSteeringError() {
  // uint16_t startTime = millis();

  //int error;
  // float prevError = 0.00F;
  // float prevIterm = 0;
  //const float kp  = 0.1; // Tune (Proportional Constant)
  // const float ki = 1; // Tune (Integral Constant)
  //const float kd = 0; // Tune (Derivative Constant)

  // uint16_t leftToFReading = this->tofLeft.getDist(); 
  // uint16_t rightToFReading = this->tofRight.getDist();

  // error = leftToFReading - rightToFReading;
  int error = getEncLeft() - getEncRight();

  // The measured distance of the tof sensors varies in short period of time
  // between +/- 5 mm, Therefore we shouldn't correct the error if it's lower than
  // this threshold

  // if (abs(error) < MIN_ERROR_THRESHOLD || abs(error) > MAX_ERROR_THRESHOLD) {
  //     motorLeft.turnForward(this->driveSpeed);
  //     motorRight.turnForward(this->driveSpeed);
  //     return;
  // }

  // unsigned long dt = millis() - this->prevTime; //millis() - startTime;
  // if (dt == 0) dt = 1;
  // this->prevTime = millis();

  int dt = 5;

  // Calculate Proportional Term
  float proportional = this->KP * error;
  // Calculate Derivative Term
  // float derivative = (this->KD *  (((float) this->prevError - (float) error) / (float) dt));
  readRawGyro();
  float derivative = this->KD * getYawRate(); 

  // btSerial.printf("Derivative: %f  Proportional: %f  Prev error: %d   Error: %d    Dt: %d", derivative, proportional, this->prevError, error, dt);

  this->prevError = error;

  // float pidTerm = proportional + integral + derivative;
  float pidTerm = proportional + derivative;

  // btSerial.printf("lm before: %d  rm before: %d  pid term: %f", this->motorLeft.getSpeed(), this->motorRight.getSpeed(), pidTerm);

  uint16_t leftMotorSpeedPid = this->motorLeft.getSpeed() - (int) pidTerm;
  uint16_t rightMotorSpeedPid = this->motorRight.getSpeed() + (int) pidTerm;

  // btSerial.printf("  ml: %d  mr: %d  pid: %f\n\n", motorLeft.getSpeed(), motorRight.getSpeed(), pidTerm);

  // btSerial.printf("new: %d  rm before: %d  pid term: %f", this->motorLeft.getSpeed(), this->motorRight.getSpeed(), motorRight.getSpeed() - (int) pidTerm, (float) motorRight.getSpeed() - pidTerm);


  // btSerial.printf("e: %d ml: %d  mr: %d  EncL: %d  EncR: %d  derivative: %f  PidTerm: %f\n", error, motorLeft.getSpeed(), motorRight.getSpeed(), getEncLeft(), getEncRight(), derivative, pidTerm);

  btSerial.printf("e: %d  EncL: %d  EncR: %d  proportional: %f  derivative: %f  PidTerm: %f\n", error, getEncLeft(), getEncRight(), proportional, derivative, pidTerm);
  Serial.printf("e: %d  EncL: %d  EncR: %d  proportional: %f  derivative: %f  PidTerm: %f\n", error, getEncLeft(), getEncRight(), proportional, derivative, pidTerm);

  // btSerial.printf("Left: %d   Right: %d   dt: %d\n", irLeft.isTriggered(), irRight.isTriggered(), dt);

  // Serial.printf("e: %d  ml: %d  mr: %d  EncL: %d  EncR: %d  PidTerm: %f\n", error, motorLeft.getSpeed(), motorRight.getSpeed(), getEncLeft(), getEncRight(), pidTerm);
  // Serial.printf("Left: %d   Right: %d   dt: %d\n", irLeft.isTriggered(), irRight.isTriggered(), dt);

  if (leftMotorSpeedPid > this->maxDriveSpeed) { // Tune
    leftMotorSpeedPid = this->maxDriveSpeed;
  }
  if (rightMotorSpeedPid > this->maxDriveSpeed)
  {
    rightMotorSpeedPid = this->maxDriveSpeed;
  }

  // Returns the turnForward or turnBackward Speed for Motors ////////
  this->motorLeft.turnForward(leftMotorSpeedPid);
  this->motorRight.turnForward(rightMotorSpeedPid);
  ////////////////////////////////////////////////////////////////////

}