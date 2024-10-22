#include <Arduino.h>
#include <string>
#include "../../include/robot.hpp"
#include "../../include/encoder.hpp"

// Robot instance
Robot robot = Robot();

// Delays
uint16_t delayTime = 500;//2 * 1000;
uint16_t startDelayTime = 2 * 1000;

// Variables for testing ToF sensors
long long counter = 0;
long long sumLeft = 0;
long long sumRight = 0;
long long sumLeftFront = 0;
long long sumRightFront = 0;

void testToFSensors() {
  uint16_t leftFrontDistance = robot.tofLeftFront.getDist();
  uint16_t rightFrontDistance = robot.tofRightFront.getDist();
  uint16_t leftDistance = robot.tofLeft.getDist();
  uint16_t rightDistance = robot.tofRight.getDist();

  sumLeftFront += leftFrontDistance;
  sumRightFront += rightFrontDistance;
  sumLeft += leftDistance;
  sumRight += rightDistance;

  counter++;

  int averageLeftFront = sumLeftFront / counter;
  int averageRightFront = sumRightFront / counter;
  int averageLeft = sumLeft / counter;
  int averageRight = sumRight / counter;

  Serial.printf("L: %d, LF: %d, RF: %d, R: %d\n", leftDistance, leftFrontDistance, rightFrontDistance, rightDistance);
  // robot.btSerial.printf("avg left front: %d\tavg right front: %d\tleft front: %d\tright front: %d\n", averageLeftFront, averageRightFront, leftFrontDistance, rightFrontDistance);
  // robot.btSerial.printf("L: %d, LF: %d, RF: %d, R: %d\n", leftDistance, leftFrontDistance, rightFrontDistance, rightDistance);
  // Serial.printf("avg left fron: %d\tavg right front: %d\tavg left: %d\tavg right: %d\n", averageLeftFront, averageRightFront, averageLeft, averageRight);
}

void testGyroData() {
  readRawGyro();
  calcGyro();
  Serial.printf("Winkelgeschwindigkeit (degree/s): %f  Winkel: %f\n", getYawRate(), getYawAngle());
  delay(50);
}

void driveClockwiseLoop() {
  if (robot.checkForStartSignal()) {
    delay(delayTime);
    robot.moveForwardUsingEncoders();
    delay(delayTime);
    robot.moveForwardUsingEncoders();
    delay(delayTime);
    robot.cellCorrectionWithToF(robot.tofLeft, robot.tofRight, robot.tofRight);
    delay(delayTime);
    robot.moveForwardUsingEncoders();
    delay(delayTime);
    robot.correctWithFrontWall();
    delay(delayTime);
    robot.correctFrontDistance();
    delay(delayTime);
    robot.correctWithFrontWall();
    delay(delayTime);
    robot.turnLeft();
  }
}

// void driveClockwiseLoop() {
//   if (robot.checkForStartSignal()) {
//     delay(delayTime);
//     robot.driveTillObstacle();
//     delay(delayTime);
//     robot.correctWithFrontWall();
//     delay(delayTime);
//     robot.turnRight(90);


//     delay(delayTime);
//     robot.driveTillObstacle();
//     delay(delayTime);
//     robot.cellCorrectionWithToF(robot.tofLeft, robot.tofRight, robot.tofRight);
//   }
// }

void tuneKD() {
  if (robot.checkForStartSignal()) {
    resetLeftEncoder();
    resetRightEncoder();
    delay(startDelayTime);
    robot.driveTillObstacle();

    robot.btSerial.printf("Finished course. Old tuning values. KP: %f, KD: %f\n", robot.KP, robot.KD);

    bool hasChanged = false;
    float kp;
    float kd;

    robot.btSerial.printf("KP: \n");

    while (!hasChanged) {
      if (robot.btSerial.available()) {
        std::string kpAsString(robot.btSerial.readStringUntil('\n').c_str());
        kp = std::stof(kpAsString);
        hasChanged = true;
      }
    }

    hasChanged = false;

    robot.btSerial.printf("KD: \n");

    while (!hasChanged) {
      if (robot.btSerial.available()) {
        std::string kdAsString(robot.btSerial.readStringUntil('\n').c_str());
        kd = std::stof(kdAsString);
        hasChanged = true;
      }
    }

    robot.KP = kp;
    robot.KD = kd;

    robot.btSerial.printf("Ready for next start! New kp: %f, New kd: %f\n", kp, kd);
  }
}

void driveCustomTrack() {
  if (robot.checkForStartSignal()) {
    resetLeftEncoder();
    resetRightEncoder();

    delay(startDelayTime);
    robot.driveTillObstacle();
    delay(delayTime);
    robot.turnRightWithGyro();
    delay(delayTime);
    robot.moveForwardUsingEncoders(2);
    delay(delayTime);
    robot.turnRightWithGyro();
    delay(delayTime);
    robot.moveForwardUsingEncoders(2);
    delay(delayTime);
    robot.turnRightWithGyro();
    delay(delayTime);
    robot.moveForwardUsingEncoders(1);
    delay(delayTime);
    robot.turnLeftWithGyro();
    delay(delayTime);
    robot.moveForwardUsingEncoders();
    delay(delayTime);
    robot.turnLeftWithGyro();
    delay(delayTime);
    robot.driveTillObstacle();
    delay(delayTime);
    robot.turnLeftWithGyro();
    delay(delayTime);
    robot.driveTillObstacle();
  }
}

void testMotors() {
  uint16_t speed = 750;

  robot.motorLeft.turnForward(speed);
  robot.motorRight.turnForward(speed);

  uint16_t startTime = millis();

  while (millis() - startTime < 3000) {
  robot.btSerial.printf("Encoder values: L: %d R: %d\n", getEncLeft(), getEncRight());
    delay(20);
  }

  robot.motorLeft.stopMotor();
  robot.motorRight.stopMotor();

  delay(500);

  robot.btSerial.printf("Final encoder values: L: %d R: %d\n", getEncLeft(), getEncRight());
}

void testMoveForward() {
  if(robot.checkForStartSignal()) {
    delay(500);
    robot.moveForwardUsingEncoders();
    delay(500);
    robot.moveForwardUsingEncoders();
    delay(500);
    robot.moveForwardUsingEncoders();
    delay(500);
    robot.moveForwardUsingEncoders();
    delay(500);
    robot.moveForwardUsingEncoders();
    delay(500);
    robot.moveForwardUsingEncoders();
    delay(500);
    robot.moveForwardUsingEncoders();
    resetLeftEncoder();
    resetRightEncoder();
  }
}

void startRun() {
  if (robot.checkForStartSignal()) {
    delay(startDelayTime);
    robot.calibrateToFOffsets();
    delay(500);
    robot.btSerial.printf("Starting floodfill\n");

    // Beginn vom Timer
    robot.sendHeadChar('1');
    delay(100);

    robot.ballPickUp();
    delay(500);
    robot.startFloodfill();

    // Enden vom TImer
    robot.sendHeadChar('3');

    // make small winner dance
    robot.motorLeft.turnForward(380);
    robot.motorRight.turnBackward(380);

    delay(20 * 1000);

    robot.motorLeft.stopMotor();
    robot.motorRight.stopMotor();

    while (1) {

    }
  }
}

void setup() {
  robot.setupRobot();
  resetLeftEncoder();
  resetRightEncoder();
  robot.btSerial.printf("SETUP\n");
}



void loop() {
  startRun();
}