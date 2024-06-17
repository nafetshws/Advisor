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
  // this->turnSpeed = 500;
  this->turnSpeed = 420;
  this->driveSpeed = 400;
  this->correctionSpeed = 550;
  this->wallDistance = 100;
  this->cellWidth = 160;
  this->tofTurnError = 10;
  this->maxDriveSpeed = 800;

  this->prevError = 0.0f;
  // this->KP = 0.4f;
  // this->KD = 0.22f;
  this->KP = 0.3f;
  this->KD = 0.35f;
  
  this->rightBrake = 0.1;
  this->leftBrake = 0.1;
  this->counterSinceLastCorrection = 0;
}

void Robot::setupRobot() {
  // SETUP SERIAL MONITOR ///////////////////////

  // start serial monitor
  Serial.begin(115200);
  // start 
  btSerial.begin("BallE BluetoothTestInterface");
  Serial.println("\nSETUP: Serial Monitor running");

  // SETUP SERVO MOTOR //////////////////////////
  setupServo(SERVO_PIN);
  delay(100);
  Serial.println("SETUP: Servo initialised");

  // SETUP MOTORS ///////////////////////////////

  // Initialise the motor objects, start pwm channels, confiure pins
  motorRight.initialise();
  motorLeft.initialise();
  // Initialise the motor encoders pins and interrupts, init timer interrupt
  initEncoders(MOTORA_ENCODER1, MOTORA_ENCODER2, MOTORB_ENCODER1, MOTORB_ENCODER2);

  Serial.println("SETUP: Motor initialised");


   // SETUP TOF //////////////////////////////////
   Serial.println("SETUP: Try to connect to TOF sensors...");

  Wire.begin();

  // init all the tof sensors
  // first two vl53L0, last two vl6180
  // initTofSensors(tofLeftFront, tofRightFront, tofLeft, tofRight);

  initTofSensors(tofLeftFront, tofRightFront, tofLeft, tofRight);

  // initTofSensors(tofLeftFront, tofRightFront, tofLeft, tofRight);
  // initTofSensors(tofLeftFront, tofRightFront);;
  // initTofSensors(tofLeft, tofRight);

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
 
  while (!irLeft.isTriggered()) {
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

void Robot::turnRightSimple(bool disableTurnErrorCorrection) {
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

void Robot::turnLeftSimple(bool disableTurnErrorCorrection) {
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


void Robot::turnRightWithGyroErrorCorrection(float degrees) {
  byte turns = 0;
  float offset = degrees;
  
  
  while(turns < 4) {
    if(offset > 0.0) {
      Serial.printf("Turn %d, this one to the right for %f degrees\n", turns, offset);
      btSerial.printf("Turn %d, this one to the right for %f degrees\n", turns, offset);
      offset = rightGyroHelper(offset);
      turns ++;
    }
    else {
      offset *= -1.0;
      Serial.printf("Turn %d, this one to the left for %f degrees\n", turns, offset);
      btSerial.printf("Turn %d, this one to the left for %f degrees\n", turns, offset);
      offset = leftGyroHelper(offset);
      turns ++;
    }
  }

  btSerial.printf("Last offset: %f\n", offset);

  resetLeftEncoder();
  resetRightEncoder();
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

  motorLeft.stopMotor();
  motorRight.stopMotor();

  // for (int i = 0; i < 50; i++) {
  //   readRawGyro();        // Get new Data
  //   calcGyro();           // Calculate new Angle

  //   btSerial.printf("Turned final %f degrees.\n", getYawAngle());
  //   delay(1);
  // }

  for (int i = 0; i < 200; i++) {
    readRawGyro();        // Get new Data
    calcGyro();           // Calculate new Angle
    delay(1);
  }

  // btSerial.printf("Turn with gyro - Actually turned by: %f\n", angleYaw);

  resetLeftEncoder();
  resetRightEncoder();
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

  for (int i = 0; i < 200; i++) {
    readRawGyro();        // Get new Data
    calcGyro();           // Calculate new Angle
    delay(1);
  }

  btSerial.printf("Turn with gyro - Turned by: %f\n", angleYaw);

  resetLeftEncoder();
  resetRightEncoder();
}


void Robot::turnLeftWithGyroErrorCorrection(float degrees) {
  byte turns = 0;
  float offset = -degrees;
  
  
  while(turns < 10) {
    if(offset > 0.0) {
      Serial.printf("Turn %d, this one to the right for %f degrees\n", turns, offset);
      btSerial.printf("Turn %d, this one to the right for %f degrees\n", turns, offset);
      offset = rightGyroHelper(offset);
      turns++;
    }
    else {
      offset *= -1.0;
      Serial.printf("Turn %d, this one to the left for %f degrees\n", turns, offset);
      btSerial.printf("Turn %d, this one to the left for %f degrees\n", turns, offset);
      offset = leftGyroHelper(offset);
      turns ++;
    }
  }

  btSerial.printf("Last offset: %f\n", offset);

  resetLeftEncoder();
  resetRightEncoder();
}

float Robot::rightGyroHelper(float degrees)
{
  setZeroAngle();

  if (degrees < 1.5) {
    return degrees;
  }

  while (getYawAngle() < (degrees - degrees * rightBrake)) {
    // btSerial.printf("Turning right: %f   Winkelgeschwindigkeit: %f\n", getYawAngle(), getYawRate());
    readRawGyro();                      // Get new Data
    calcGyro();                         // Calculate new Angle
    motorRight.turnBackward(turnSpeed); // Turn Mouse
    motorLeft.turnForward(turnSpeed);
  }

  motorLeft.stopMotor();
  motorRight.stopMotor();

  btSerial.printf("Stopped the motors at %f. Should turn %f. Brake was %f. Now Calculating Angle for a sec. \n", getYawAngle(), degrees, rightBrake);

  readRawGyro(); // Get new Data
  calcGyro();    // Calculate new Angle

  for (int i = 0; i < 100; i++)
  {
    readRawGyro(); // Get new Data
    calcGyro();    // Calculate new Angle
    delay(3);
  }

  Serial.printf("Final Turn: %f degrees.\n", getYawAngle());

  if (degrees > 30 && abs(getYawAngle() - degrees) > 1.0)
  {
    float temp = getYawAngle() - degrees;
    if(temp > 0.0) {
      rightBrake += 0.02;
    }
    else {
      rightBrake -= 0.02;
    }

    Serial.printf("Right Brake value changed to %f. \n", rightBrake);
  }

  return degrees - getYawAngle();
}

float Robot::leftGyroHelper(float degrees) {
  
  setZeroAngle();
  if(degrees < 1.5) {
    return degrees;
  }

  while(getYawAngle() > (-degrees + degrees * leftBrake)) {
    //  btSerial.printf("Turning Left: %f   Winkelgeschwindigkeit: %f\n", getYawAngle(), getYawRate());
        readRawGyro();        // Get new Data
        calcGyro();           // Calculate new Angle
        motorRight.turnForward(turnSpeed); // Turn Mouse
        motorLeft.turnBackward(turnSpeed);
      }
      motorLeft.stopMotor();
      motorRight.stopMotor();

      btSerial.printf("Stopped the motors at %f. Should turn %f. Brake was %f. Now Calculating Angle for a sec. \n", getYawAngle(), degrees, leftBrake);

      readRawGyro();        // Get new Data
      calcGyro();           // Calculate new Angle
          
      for(int i = 0; i < 100; i++) {
        readRawGyro();        // Get new Data
        calcGyro();           // Calculate new Angle
        delay(3);
      }

      Serial.printf("Final Turn: %f degrees.\n", getYawAngle());

      if(degrees > 30 && abs((-getYawAngle()) - degrees) > 1.0) {
        float temp = getYawAngle() + degrees;
        if((-getYawAngle()) - degrees > 0.0) {
          leftBrake += 0.02;
        }
        else {
          leftBrake -= 0.02;
        }
        Serial.printf("Left Brake value changed to %f. \n", leftBrake);
      }

  return (-getYawAngle()) - degrees;
  

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
      this->turnRightSimple(true);
      this->turnTime = turnTimeTmp;
    } else {
      // Turn left
      uint16_t turnTimeTmp = this->turnTime;
      // rotation time is proportional to the distance difference between the tof sensors
      this->turnTime = abs(distanceDifference) / 4;
      // Calls turnLeft(), but disables error correction to avoid infinite recursion
      this->turnLeftSimple(true);
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
  // this->btSerial.println("Front Wall");
  // bool hasWall;
  // bool hasChanged = false;
  // while (!hasChanged) {
  //   if (this->btSerial.available()) {
  //     std::string s(btSerial.readStringUntil('\n').c_str());
  //     hasWall = stoi(s);
  //     hasChanged = true;
  //   }
  // }

  uint16_t averageDistance = calcAverageDistance(tofLeftFront, 3);
  bool hasWall = averageDistance < this->wallDistance;

  // bool hasWall = tofLeftFront.getDist() < this->wallDistance && tofRightFront.getDist() < this->wallDistance;
  // hasWall = tofRightFront.getDist() < this->wallDistance;
  this->btSerial.printf("front wall measured: %d - Distance: %d\n", hasWall, averageDistance);
  // return hasWall;
  // bool hasWall = irLeft.isTriggered() && irRight.isTriggered();
  // this->btSerial.printf("Front wall measured: %d\n", hasWall);
  return hasWall;
}

bool Robot::wallRight() {
  //  this->btSerial.println("Right Wall");
  // bool hasWall;
  // bool hasChanged = false;
  // while (!hasChanged) {
  //   if (this->btSerial.available()) {
  //     std::string s(btSerial.readStringUntil('\n').c_str());
  //     hasWall = stoi(s);
  //     hasChanged = true;
  //   }
  // }

  uint16_t averageDistance = calcAverageDistance(tofRight, 3);
  bool hasWall = averageDistance < this->wallDistance;
  this->btSerial.printf("Right wall measured: %d - Distance: %d\n", hasWall, averageDistance);
  return hasWall;
}

bool Robot::wallLeft() {
// this->btSerial.println("Left Wall");
// bool hasWall;
// bool hasChanged = false;
//   while (!hasChanged) {
//     if (this->btSerial.available()) {
//       std::string s(btSerial.readStringUntil('\n').c_str());
//       hasWall = stoi(s);
//       hasChanged = true;
//     }
//   }

  uint16_t averageDistance = calcAverageDistance(tofLeft, 3);
  bool hasWall = averageDistance < this->wallDistance;
  this->btSerial.printf("Left wall measured: %d - Distance: %d\n", hasWall, averageDistance);
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

  float wheelRotationsNeeded = (distance * 16.) / WHEEL_CIRCUMFERENCE;
  // Encoder increments 3 times per motor revolution * 30 (Gear ratio) = 90
  float motorRotationsNeeded = wheelRotationsNeeded * 90;

  uint16_t speedDelta = this->driveSpeed / 5;

  // Accelerate over a time frame of 100 ms to the desired speed
  // for (int i = 0; i < 5; i++) {
  //   motorLeft.turnForward(motorLeft.getSpeed()   + speedDelta);
  //   motorRight.turnForward(motorRight.getSpeed() + speedDelta);
  //   delay(20);
  // }

  // if (this->lastPDTerms[0] = 0) {

  motorLeft.turnForward(driveSpeed);
  motorRight.turnForward(driveSpeed);
  // }

  // Move forward until distance is covered, while correcting the steering error
  do {
      this->correctSteeringError();
      delay(5);
  } while (getEncLeft()  - startValueEncLeft  < motorRotationsNeeded && 
           getEncRight() - startValueEncRight < motorRotationsNeeded);

  motorLeft.stopMotor();
  motorRight.stopMotor();

  // resetLeftEncoder();
  // resetRightEncoder();

  delay(1000);
}

void Robot::alignRobot(TOF_6180 &tof1, TOF_6180 &tof2) {
  int16_t difference = calcAverageDifference(tof1, tof2, 2);

  while (abs(difference) > 2) {
    btSerial.printf("Difference: %d\n", difference);
    unsigned long startTime = micros();

    // If the difference is negative, turn left else right
    if (difference < 0) {
      motorLeft.turnBackward(correctionSpeed);
      motorRight.turnForward(correctionSpeed);
    } else {
      motorRight.turnBackward(correctionSpeed);
      motorLeft.turnForward(correctionSpeed);
    }

    // TODO: update value late
    while (micros() - startTime < 20 * 1e3) {
      
    }

    motorLeft.stopMotor();
    motorRight.stopMotor();

    difference = this->calcAverageDifference(tofLeftFront, tofRightFront, 2);
  }
}

void Robot::driveToMiddle(TOF_6180 &l1, TOF_6180 &l2, TOF_6180 &r1, TOF_6180 &r2) {
  uint16_t timeToCorrect = 300;

  int16_t difference = calcAverageDifference(l1, r1, 3);
  while(abs(difference) > 10) {
    // Initiate correction

    // Robot is too much to the right if positive -> correct left
    if (difference < 0) {
      uint32_t startTime = millis();

      while (millis() - startTime < timeToCorrect) {
        motorRight.turnBackward(turnSpeed);
      }
      motorRight.stopMotor();

      startTime = millis();

      while (millis() - startTime < timeToCorrect) {
        motorLeft.turnBackward(turnSpeed);
      }
      motorLeft.stopMotor();
    } else {
      uint32_t startTime = millis();

      while (millis() - startTime < timeToCorrect) {
        motorLeft.turnBackward(turnSpeed);
      }
      motorLeft.stopMotor();

      startTime = millis();

      while (millis() - startTime < timeToCorrect) {
        motorRight.turnBackward(turnSpeed);
      }
      motorRight.stopMotor();
    }

    // Drive forward again
    int16_t startTime = millis();
    while (millis() - startTime < timeToCorrect) {
      motorLeft.turnForward(turnSpeed);
      motorRight.turnForward(turnSpeed);
    }

    motorLeft.stopMotor();
    motorRight.stopMotor();

    difference = calcAverageDifference(l1, r1, 2);
  }
}

uint16_t Robot::calcAverageDistance(TOF_6180 &tof, int samples) {
  uint16_t distance = 0; 

  for (int i = 0; i < samples; i++) {
    uint16_t measuredDistance = tof.getDist();
    // distance += tof.getDist();
    distance += measuredDistance; 
  }

  return distance / samples;
}

void Robot::correctFrontDistance() {
  uint16_t distance = calcAverageDistance(tofLeftFront, 3);

  while (distance > 67 || distance < 63) {
    unsigned long startTime = micros();

    if (distance < 65) {
      // Drive back
      motorLeft.turnBackward(correctionSpeed);
      motorRight.turnBackward(correctionSpeed);
    } else {
      // Drive forward
      motorLeft.turnForward(correctionSpeed);
      motorRight.turnForward(correctionSpeed);
    }

    while (micros() - startTime < 20 * 1e3) {

    }

    motorLeft.stopMotor();
    motorRight.stopMotor();

    distance = calcAverageDistance(tofLeftFront, 2);
  }

}

/*
* The error is corrected in 3 stages:
*   1. Turn until facing right or left side wall 
*   2. Drive to the middle
*   3. Turn to previous drive direction 
*/
void Robot::cellCorrectionWithToF(TOF_6180 &l1, TOF_6180 &r1, TOF_6180 &r2) {
  uint16_t maxWallDistance = 100;
  boolean useLeftWall = false;
  // There is no wall on the left
  if (l1.getDist() <= maxWallDistance) {
    useLeftWall = true;
    // Step 1
    turnLeft(90);
  } else {
    turnRight(90);
  }

  delay(300);

  // alignRobot(tofLeftFront, tofRightFront);
  this->correctWithFrontWall();

  delay(300);

  // Step 2
  this->correctFrontDistance();
  delay(300);
  this->correctWithFrontWall();
  delay(300);


  // uint16_t distance = calcAverageDistance(tofLeftFront, 3);
  // while (distance > 62 || distance < 58) {
  //   unsigned long startTime = micros();

  //   if (distance < 60) {
  //     // Drive back
  //     motorLeft.turnBackward(correctionSpeed);
  //     motorRight.turnBackward(correctionSpeed);
  //   } else {
  //     // Drive forward
  //     motorLeft.turnForward(correctionSpeed);
  //     motorRight.turnForward(correctionSpeed);
  //   }

  //   while (micros() - startTime < 20 * 1e3) {

  //   }

  //   motorLeft.stopMotor();
  //   motorRight.stopMotor();

  //   distance = calcAverageDistance(tofLeftFront, 2);
  // }

  // delay(500);
  // this->correctWithFrontWall();
  // this->correctFrontDistance();
  // delay(500);

  // Step 3
  if (useLeftWall) {
    turnRight(90);
  } else {
    turnLeft(90);
  }
}


void Robot::correctRobot(boolean isWallFront, boolean isWallLeft, boolean isWallRight) {
  if (isWallFront && isWallLeft && isWallRight) {
    this->correctWithFrontWall();
    delay(300);
    this->correctFrontDistance();
    delay(300);
    this->correctWithFrontWall();
    delay(300);
    this->cellCorrectionWithToF(tofLeft, tofRight, tofRight);
    delay(300);
  } else if (isWallFront) {
    this->correctWithFrontWall();
    delay(300);
    this->correctFrontDistance();
    delay(300);
    this->correctWithFrontWall();
    delay(300);
  } else if ((isWallLeft || isWallRight) && counterSinceLastCorrection > 3) {
    this->cellCorrectionWithToF(tofLeft, tofRight, tofRight);
    delay(300);
  } else {
    counterSinceLastCorrection++;
    return;
  }
  
  counterSinceLastCorrection = 0;
}


int16_t Robot::calcAverageDifference(TOF_6180 &tof1, TOF_6180 &tof2, int samples) {
  uint16_t leftFrontDistance = tof1.getDist();
  uint16_t rightFrontDistance = tof2.getDist();

  for (int i = 0; i < samples - 1; i++) {
     leftFrontDistance += tof1.getDist(); 
     rightFrontDistance += tof2.getDist();

    //  delay(50);
  }

  leftFrontDistance /= samples;
  rightFrontDistance /= samples;

  return leftFrontDistance - rightFrontDistance;
}

void Robot::correctWithFrontWall() {
  float TURN_CORRECTION_CONSTANT = 0.9;

  int16_t difference = this->calcAverageDifference(tofLeftFront, tofRightFront); 

  // btSerial.printf("Turning for %f ms. Difference: %d\n", difference * TURN_CORRECTION_CONSTANT, difference);

  for (int i = 0; i < 1000; i++) {
    // Abbort correction if the margin of error is too small 
    if (abs(difference) <= 2) {
      return;
    } else if (abs(difference) < 20) {
        // btSerial.printf("Starting fine adjustement\n");

        while (abs(difference) > 2) {
          // btSerial.printf("Difference: %d\n", difference);
          unsigned long startTime = micros();

          // If the difference is negative, turn left else right
          if (difference < 0) {
            motorLeft.turnBackward(correctionSpeed);
            motorRight.turnForward(correctionSpeed);
          } else {
            motorRight.turnBackward(correctionSpeed);
            motorLeft.turnForward(correctionSpeed);
          }

          while (micros() - startTime < 30 * 1e3) {
            
          }

          motorLeft.stopMotor();
          motorRight.stopMotor();

          difference = this->calcAverageDifference(tofLeftFront, tofRightFront, 2);
        }

        return;
    } 

    // If the difference is negative, turn left else right
    if (difference < 0) {
      motorLeft.turnBackward(turnSpeed);
      motorRight.turnForward(turnSpeed);
    } else {
      motorRight.turnBackward(turnSpeed);
      motorLeft.turnForward(turnSpeed);
    }

    // btSerial.printf("Turning for %f ms. Difference: %d\n", difference * TURN_CORRECTION_CONSTANT, difference);

    uint16_t startTime = millis();

    while (millis() - startTime < abs(difference) * TURN_CORRECTION_CONSTANT) {

    }

    motorLeft.stopMotor();
    motorRight.stopMotor();

    difference = this->calcAverageDifference(tofLeftFront, tofRightFront); 
  }

  resetLeftEncoder();
  resetRightEncoder();
}

void Robot::turnLeft(float degrees) {
  turnLeftWithGyro(degrees);

  // btSerial.printf("Turned: %f degrees\n", getYawAngle());

  if (getYawAngle() + degrees > 0) {
    // Didn't turn far enough -> turn left further
    smallAdjustmentGyro(getYawAngle() + degrees, true);
  } else {
    smallAdjustmentGyro(abs(getYawAngle()) - degrees, false);
  }

} 

void Robot::turnRight(float degrees) {
  turnRightWithGyro(degrees);

  // btSerial.printf("Turned: %f degrees\n", getYawAngle());

  if (degrees - getYawAngle() > 0) {
    // Didn't turn far enough -> turn right further
    // btSerial.printf("Correcting further to the right\n");
    smallAdjustmentGyro(degrees - getYawAngle(), false);
  } else {
    // btSerial.printf("Correcting further to the left\n");
    smallAdjustmentGyro(getYawAngle() - degrees, true);
  }
} 

void Robot::smallAdjustmentGyro(float degrees, bool turnLeft) {
  setZeroAngle();

  readRawGyro();
  calcGyro();

  while (abs(getYawAngle()) < degrees - 0.8) {
    unsigned long startTime = micros();

    if (turnLeft) {
      motorLeft.turnBackward(correctionSpeed);
      motorRight.turnForward(correctionSpeed);
    } else {
      motorRight.turnBackward(correctionSpeed);
      motorLeft.turnForward(correctionSpeed);
    }

    while (micros() - startTime < 20 * 1e3) {
      readRawGyro();
      calcGyro();
    }

    motorLeft.stopMotor();
    motorRight.stopMotor();

    // difference = this->calcAverageDifference(tofLeftFront, tofRightFront, 2);
    for (int i = 0; i < 50; i++) {
      readRawGyro();
      calcGyro();
      delay(1);
    }

    // btSerial.printf("Angle turned: %f\n", getYawAngle());
  }
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
  // btSerial.printf("e: %d  EncL: %d  EncR: %d  proportional: %f  derivative: %f  PidTerm: %f\n", error, getEncLeft(), getEncRight(), proportional, derivative, pidTerm);
  // Serial.printf  ("e: %d  EncL: %d  EncR: %d  proportional: %f  derivative: %f  PidTerm: %f\n", error, getEncLeft(), getEncRight(), proportional, derivative, pidTerm);

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

void Robot::ballPickUp() {
  delay(500);
  moveForwardUsingEncoders(1);
  delay(500);
  turnRight(90);
  delay(1000);
  moveForwardUsingEncoders(1);
  delay(1000);
  turnLeft(45);
  delay(1000);

  // Insert logic for servo //////
  servoDown();
  delay(500);
  uint16_t startTime = millis();

  // Drive to ball
  motorLeft.turnForward(driveSpeed);
  motorRight.turnForward(driveSpeed);

  while (millis() - startTime < 330) {
    
  }

  motorLeft.stopMotor();
  motorRight.stopMotor();

  delay(500);
  servoUp();
  delay(2000);

  // Drive back
  motorLeft.turnBackward(driveSpeed);
  motorRight.turnBackward(driveSpeed);

  // startTime = millis();

  while (millis() - startTime < 330) {
    
  }

  motorLeft.stopMotor();
  motorRight.stopMotor();
  ////////////////////////////////

  turnLeft(135);
  delay(300);
  moveForwardUsingEncoders();
  delay(300);
  correctWithFrontWall();
  delay(300);
  correctFrontDistance();
  delay(300);
  correctWithFrontWall();
  delay(300);
  turnLeft();
  delay(300);
  moveForwardUsingEncoders();
  delay(300);
  correctWithFrontWall();
  delay(300);
  correctFrontDistance();
  delay(300);
  correctWithFrontWall();
  delay(300);
  turnRight();
  correctWithFrontWall();
  delay(300);
  correctFrontDistance();
  delay(300);
  correctWithFrontWall();
  delay(300);
  turnRight();

}