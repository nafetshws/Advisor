#include <Arduino.h>
#include "../../include/robot.hpp"

Robot robot = Robot();
uint16_t delayTime = 2 * 1000;
uint16_t startDelayTime = 8 * 1000;

long long counter = 0;
long long sumLeft = 0;
long long sumRight = 0;

void setup() {
  robot.setupRobot();
}


void loop() {
  // uint16_t leftFrontDistance = robot.tofLeftFront.getDist();
  // uint16_t rightFrontDistance = robot.tofRightFront.getDist();

  // sumLeft += leftFrontDistance;
  // sumRight += rightFrontDistance;
  // counter++;

  // int averageLeft = sumLeft / counter;
  // int averageRight = sumRight / counter;

  // Serial.printf("TOF left: %d\tTOF right: %d\tSensor difference: %d\tAverage left: %d\tAverage right:%d\n", leftFrontDistance, rightFrontDistance, leftFrontDistance - rightFrontDistance, averageLeft, averageRight);

  // delay(100);

  if (robot.checkForStartSignal()) {
    delay(startDelayTime);
    robot.driveTillObstacle();
    delay(delayTime);
    robot.turnRight();
    delay(delayTime);
    robot.driveTillObstacle();
    delay(delayTime);
    robot.turnRight();

    while (1) {
      Serial.printf("LOOPING EXITING\n");
      delay(100);
    }
  }


  // if (robot.checkForStartSignal()) {
  //   delay(startDelayTime);
  //   robot.driveTillObstacle();
  //   delay(delayTime);
  //   robot.turnRight();
  //   delay(delayTime);
  //   robot.driveTillObstacle();
  //   delay(delayTime);
  //   robot.turnRight();
  //   delay(delayTime);
  //   robot.driveTillObstacle();
  //   delay(delayTime);
  //   robot.turnRight();
  //   delay(delayTime);
  //   robot.driveTillObstacle();
  //   while (1) {
  //     delay(delayTime);
  //     robot.turnRight();
  //     delay(delayTime);
  //     robot.driveTillObstacle();
  //   }

  //   while (1) {
  //     Serial.printf("LOOPING EXITING\n");
  //   }
  // }
  // return;
}