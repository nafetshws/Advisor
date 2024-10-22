#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <cstdint>
#include <Arduino.h>

// Prescalar so timer gets updated every 1µs
#define TIMER0_PRESCALAR 80
// Sets off the intterupt ever 100 ms = 1µs * TIMERO_ALARM_AT
#define TIMER0_ALARM_AT 100000
// Specifies how often the interrupt is triggerd in one second
#define INTERUPTS_PER_SECOND 10

// Timer object for coniguration, static for encoder.cpp use only
// static hw_timer_t *Timer0_Cfg;

// Stores the counted rotations of motor right
static uint32_t ENC_RIGHT_COUNT;
// Stores the counted rotations of motor left
static uint32_t ENC_LEFT_COUNT;

// Stores the rps value of motor A/B, static for encoder.cpp use only
// static volatile uint16_t rpsA;
// static volatile uint16_t rpsB;

// Stores the dir of motor A, 1 forward, -1 backward, static for encoder.cpp use only
// static volatile int8_t dirA;
// static volatile int8_t dirB;

// Stores the encoder 2 pins to determand the direction
static uint8_t pinEncoderA2;
static uint8_t pinEncoderB2;


/**
 * @brief   Interrupt Service Routine for the Timer0 for calculating
 *          the rps of the motors
*/
// void Timer0_ISR();


/**
 * @brief   Interrupt Service Routine for couting up the motor A rotations
*/
void EncoderA01_ISR();

/**
 * @brief   Interrupt Service Routine for couting up the motor B rotations
*/
void EncoderB01_ISR();


/**
 * @brief   Delcares Pinmodes of Encoders and starts the Timer0 to calc the rps of motors
 * @param   encA01  Pin of Motor A Encoder Channel 1
 * @param   encA02  Pin of Motor A Encoder Channel 2
 * @param   encB01  Pin of Motor B Encoder Channel 1
 * @param   encB02  Pin of Motor B Encoder Channel 2
*/
void initEncoders(uint8_t encA01, uint8_t encA02, uint8_t encB01, uint8_t encB02);


/**
 * @brief   Returns the speed of motor A in rounds per second
 * @return  rounds per second
*/
// uint16_t getRpsMotorA();


/**
 * @brief   Returns the speed of motor B in rounds per second
 * @return  rounds per second
*/
// uint16_t getRpsMotorB();

/**
 * @brief   Returns the dir * speed of motor A in rounds per second
 *          +values for forwards, -values for backwards
 * @return  dir * rounds per second
*/
// int16_t getDirMotorA();


/**
 * @brief   Returns the dir * speed of motor B in rounds per second
 *          +values for forwards, -values for backwards
 * @return  dir * rounds per second
*/
// int16_t getDirMotorB();


uint32_t getEncRight();

uint32_t getEncLeft();

void resetLeftEncoder();
void resetRightEncoder();

void setLeftEncoder(uint32_t newValue);
void setRightEncoder(uint32_t newValue);

#endif