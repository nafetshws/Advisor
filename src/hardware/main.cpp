#include <Arduino.h>
#include <string>
#include "../../include/robot.hpp"
#include "../../include/encoder.hpp"

Robot robot = Robot();
uint16_t delayTime = 2 * 1000;
uint16_t startDelayTime = 3 * 1000;

long long counter = 0;

long long sumLeft = 0;
long long sumRight = 0;
long long sumLeftFront = 0;
long long sumRightFront = 0;

uint32_t leftEncoder; 
uint32_t rightEncoder;

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

  Serial.printf("avg left front: %d\tavg right front: %d\tavg left: %d\tavg right: %d\n", averageLeftFront, averageRightFront, averageLeft, averageRight);
}

void setup() {
  robot.setupRobot();
  resetLeftEncoder();
  resetRightEncoder();
  initIMU(2000);
  delay(1000);
}


void loop() {
  // readRawGyro();
  // calcGyro();
  // Serial.printf("Winkelgeschwindigkeit (degree/s): %f  Winkel: %f\n", getYawRate(), getYawAngle());
  // delay(50);

  if (robot.checkForStartSignal()) {
    delay(delayTime);
    robot.driveTillObstacle();
    delay(delayTime);
    robot.turnGyroRight();
  }

  // if (robot.checkForStartSignal()) {
  //   resetLeftEncoder();
  //   resetRightEncoder();
  //   delay(startDelayTime);
  //   robot.driveTillObstacle();
  //   delay(delayTime);
  //   robot.turnGyroRight();
  //   delay(delayTime);
  //   robot.moveForwardUsingEncoders(2);
  //   delay(delayTime);
  //   robot.turnGyroRight();
  //   delay(delayTime);
  //   robot.moveForwardUsingEncoders(2);
  //   delay(delayTime);
  //   robot.turnGyroRight();
  //   delay(delayTime);
  //   robot.moveForwardUsingEncoders(1);
  //   delay(delayTime);
  //   robot.turnGyroLeft();
  //   delay(delayTime);
  //   robot.moveForwardUsingEncoders();
  //   delay(delayTime);
  //   robot.turnGyroLeft();
  //   delay(delayTime);
  //   robot.driveTillObstacle();
  //   delay(delayTime);
  //   robot.turnGyroLeft();
  //   delay(delayTime);
  //   robot.driveTillObstacle();

  //   robot.btSerial.printf("Finished course. Old tuning values. KP: %f, KD: %f\n", robot.KP, robot.KD);

  //   bool hasChanged = false;
  //   float kp;
  //   float kd;

  //   robot.btSerial.printf("KP: \n");

  //   while (!hasChanged) {
  //     if (robot.btSerial.available()) {
  //       std::string kpAsString(robot.btSerial.readStringUntil('\n').c_str());
  //       kp = std::stof(kpAsString);
  //       hasChanged = true;
  //     }
  //   }

  //   hasChanged = false;

  //   robot.btSerial.printf("KD: \n");

  //   while (!hasChanged) {
  //     if (robot.btSerial.available()) {
  //       std::string kdAsString(robot.btSerial.readStringUntil('\n').c_str());
  //       kd = std::stof(kdAsString);
  //       hasChanged = true;
  //     }
  //   }

  //   robot.KP = kp;
  //   robot.KD = kd;

  //   // while (robot.checkForStartSignal()) {
  //   //   delay(100);
  //   // }

  //   robot.btSerial.printf("Ready for next start! New kp: %f, New kd: %f\n", kp, kd);
  // }






  // if (!robot.checkForStartSignal()) {
  //   return;
  // }

  // delay(startDelayTime);

  // robot.btSerial.printf("Left enc: %d Right enc: %d Difference: %d\n", leftEncoder, rightEncoder, leftEncoder - rightEncoder);

  // resetLeftEncoder();
  // resetRightEncoder();

  // robot.motorLeft.turnForward(50);
  // robot.motorRight.turnForward(50);

  // do {
  //   leftEncoder = getEncLeft();
  //   rightEncoder = getEncRight();
  // } while (leftEncoder < 90 || rightEncoder < 90); 

  // robot.motorLeft.stopMotor();
  // robot.motorRight.stopMotor();

  // Serial.printf("Left enc: %d\tRight enc: %d\tDifference: %d\n", leftEncoder, rightEncoder, leftEncoder - rightEncoder);
  // robot.btSerial.printf("Left enc: %d Right enc: %d Difference: %d\n", leftEncoder, rightEncoder, leftEncoder - rightEncoder);
}