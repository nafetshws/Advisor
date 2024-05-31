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
}