#include <Arduino.h>
#include "../../include/robot.hpp"

Robot robot = Robot();
uint16_t delayTime = 2 * 1000;
uint16_t startDelayTime = 3 * 1000;

void setup() {
  robot.setupRobot();
}

void loop() {
  if (robot.checkForStartSignal()) {
    delay(startDelayTime);
    robot.driveTillObstacle();
    delay(delayTime);
    robot.turnRight();
    delay(delayTime);
    robot.driveTillObstacle();
    delay(delayTime);
    robot.turnRight();
    delay(delayTime);
    robot.driveTillObstacle();
    delay(delayTime);
    robot.turnRight();
    delay(delayTime);
    robot.driveTillObstacle();
    while (1) {
      delay(delayTime);
      robot.turnRight();
      delay(delayTime);
      robot.driveTillObstacle();
    }

    while (1) {
      Serial.printf("LOOPING EXITING\n");
    }
  }
  return;
}