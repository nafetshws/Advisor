#include <Arduino.h>
#include "../../include/robot.hpp"

Robot robot = Robot();
uint16_t delayTime = 2 * 1000;
uint16_t startDelayTime = 3 * 1000;

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

  Serial.printf("avg left front: %d\tavg right front: %d\tavg left: %d\tavg right: %d\n", averageLeftFront, averageRightFront, averageLeft, averageRight);
}

void setup() {
  robot.setupRobot();
}


void loop() {
  if (robot.checkForStartSignal()) {
    delay(startDelayTime);
    robot.driveTillObstacle();

    printf("finished\n");

    while (1) {
      delay(100); 
    }
  }

  // uint16_t leftFrontDistance = robot.tofLeftFront.getDist();
  // uint16_t rightFrontDistance = robot.tofRightFront.getDist();

  // sumLeft += leftFrontDistance;
  // sumRight += rightFrontDistance;
  // counter++;

  // int averageLeft = sumLeft / counter;
  // int averageRight = sumRight / counter;

  // Serial.printf("TOF left: %d\tTOF right: %d\tSensor difference: %d\tAverage left: %d\tAverage right:%d\n", leftFrontDistance, rightFrontDistance, leftFrontDistance - rightFrontDistance, averageLeft, averageRight);

  // delay(100);

  // if (robot.checkForStartSignal()) {
  //   delay(startDelayTime);
    // robot.moveForward(3);
    // robot.driveTillObstacle();
    // Serial.printf("Finished");
    // delay(delayTime);
    // robot.moveForward(1);
    // delay(delayTime);
    // robot.moveForward(1);
    // delay(startDelayTime);
    // robot.driveTillObstacle();
    // delay(delayTime);
    // robot.turnRight();
    // delay(delayTime);
    // robot.driveTillObstacle();
    // delay(delayTime);
    // robot.turnRight();

    // while (1) {
    //   // Serial.printf("LOOPING EXITING\n");
    //   delay(100);
    // }
  // }


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