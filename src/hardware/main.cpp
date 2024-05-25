#include <Arduino.h>
#include "../../include/robot.hpp"

Robot robot = Robot();

void setup() {
  robot.setupRobot();
}

void loop() {
  if (robot.checkForStartSignal()) {
    delay(5 * 1000);
    robot.driveTillObstacle();
    delay(2 * 1000);
    robot.turnRight();
    delay(2 * 1000);
    robot.driveTillObstacle();
    delay(2 * 1000);
    robot.turnRight();
    delay(2 * 1000);
    robot.driveTillObstacle();
    delay(2 * 1000);
    robot.turnRight();
    delay(2 * 1000);
    robot.driveTillObstacle();
    while (1) {
      delay(2 * 1000);
      robot.turnRight();
      delay(2 * 1000);
      robot.driveTillObstacle();
    }

    while (1) {
      Serial.printf("LOOPING EXITING\n");
    }
  }
  return;
}