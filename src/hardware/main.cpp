#include <Arduino.h>
#include "../../include/motor.hpp"
#include "../../include/encoder.hpp"
#include "../../include/tof.hpp"


// ESP32 PIN OUT ////////////////////////////////

// PINS MOTOR A
#define MOTORA_IN1  0
#define MOTORA_IN2  2
#define MOTORA_PWM  15
#define MOTORA_ENCODER1 36
#define MOTORA_ENCODER2 39
// PINS MOTOR B
#define MOTORB_IN1  4
#define MOTORB_IN2  16
#define MOTORB_PWM  17
#define MOTORB_ENCODER1 35
#define MOTORB_ENCODER2 34

// PINS TOF SENSORS
#define TOF1_SHT_PIN 32
#define TOF2_SHT_PIN 33
#define TOF3_SHT_PIN 25
#define TOF4_SHT_PIN 26
#define TOF5_SHT_PIN 27
#define TOF6_SHT_PIN 14


// Global Motor Objects
Motor motorA = Motor(MOTORA_IN1, MOTORA_IN2, MOTORA_PWM, MOTORA_PWM_CHANNEL);
Motor motorB = Motor(MOTORB_IN1, MOTORB_IN2, MOTORB_PWM, MOTORB_PWM_CHANNEL);

// TOF Sensor Objects
TOF tofLeft =         TOF(1, TOF_START_ADDR + 0, TOF1_SHT_PIN);
TOF tofLeftFront =    TOF(2, TOF_START_ADDR + 1, TOF2_SHT_PIN);
TOF tofRightFront =   TOF(3, TOF_START_ADDR + 2, TOF3_SHT_PIN);
TOF tofRight =        TOF(4, TOF_START_ADDR + 3, TOF4_SHT_PIN);
TOF tofLeft45 =       TOF(5, TOF_START_ADDR + 4, TOF5_SHT_PIN);
TOF tofRight45 =      TOF(6, TOF_START_ADDR + 5, TOF6_SHT_PIN);


void move_forward(double distance = 1) { 

  uint16_t current = (tofRightFront.getDist() + tofLeftFront.getDist())/2; 
  motorA.turnForward(50); 
  motorB.turnForward(50);  

  while(current - tofRight.getDist() < 16*distance) {}

  motorA.stopMotor();
  motorB.stopMotor(); 

}

uint32_t prev_angle, current_angle;

void turn_right() {
  read_encoders();
  current_angle = Encoders[2];
  prev_angle = current_angle;
  uint32_t angle_turned = 0;
    motorA.turnForward(50);
    motorB.turnBackward(50);
  while(angle_turned%360 < 90) {
    read_encoders();
    current_angle = Encoders[2];
    angle_turned = current_angle - prev_angle;
  }
  motorA.stopMotor();
  motorB.stopMotor(); 
}

void turn_left() {
  read_encoders();
  current_angle = Encoders[2];
  prev_angle = current_angle;
  uint32_t angle_turned = 0;
  motorB.turnForward(50); // Tune
  motorA.turnBackward(50);
  while(angle_turned%360 < 90) {
    read_encoders();
    current_angle = Encoders[2];
    angle_turned = current_angle - prev_angle;
  }
  motorA.stopMotor();
  motorB.stopMotor(); 
}







void setup() {

  // SETUP SERIAL MONITOR ///////////////////////

  // start serial monitor
  Serial.begin(115200);
  Serial.println("\nSETUP: Serial Monitor running");



  // SETUP MOTORS ///////////////////////////////

  // Initialise the motor objects, start pwm channels, confiure pins
  motorA.initialise();
  motorB.initialise();
  // Initialise the motor encoders pins and interrupts, init timer interrupt
  initEncoders(MOTORA_ENCODER1, MOTORA_ENCODER2, MOTORB_ENCODER1, MOTORB_ENCODER2);

  Serial.println("SETUP: Motor initialised");



  // SETUP TOF //////////////////////////////////

  Serial.println("SETUP: Try to connect to TOF sensors...");

  // init all the tof sensors
  initTofSensors(tofLeft, tofLeftFront, tofRightFront, tofRight, tofLeft45, tofRight45);

  Serial.println("SETUP: TOF Sensors initialised");

  // SETUP END //////////////////////////////////
  Serial.println("SETUP: Setup Done");
  Serial.println("Start Looping:");
}

void loop() {

  // Read out Tof Sensor data
  Serial.printf("Distanz von 1,2,3:\t%d\t%d\t%d\n", tofLeft.getDist(), tofLeftFront.getDist(), tofRightFront.getDist());
  delay(1000);

}