#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstdint>

// ESP PWM Channels
// servo motor always use channel 1
#define MOTORA_PWM_CHANNEL 2
#define MOTORB_PWM_CHANNEL 3
// ESP PWM Frequency 4kHz
#define MOTOR_PWM_FREQUENCY 4000
// ESP PWM RESOLUTION 8 Bit
#define MOTOR_PWM_RESOLUTION 8


class Motor {

public:
    Motor();

    /**
     * @brief   Creates a motor object for the TB6612FNG motor driver
     * @param   pinA        Pin for forward direction
     * @param   pinB        Pin for backwards direction
     * @param   pinPWM      Pin for enable/disable/speed control
     * @param   pwmChannel  Internal ESP32 pwm channel
    */
    Motor (uint8_t pinA, uint8_t pinB, uint8_t pinPWM, uint8_t pwmChannel);


    /** 
     * @brief   Initialise the motor objects, start pwm channels, confiure pins
    */
    void initialise();


    /**
     * @brief   Inifitly turns the motor forward with specified speed
     * @param   speed       motor speed, 8 bit resolution, 0 -> STOP 255 -> FULLSPEED
    */
    void turnForward(uint8_t speed);


    /**
     * @brief   Inifitly turns the motor backwards with specified speed
     * @param   speed       motor speed, 8 bit resolution, 0 -> STOP 255 -> FULLSPEED
    */
    void turnBackward(uint8_t speed);

    /**
     * @brief   Returns currents set speed of motor
    */
    uint8_t getSpeed();


    /**
     * @brief   Stops the motor immediatly
    */
    void stopMotor();
    

private:

    uint8_t pinA;           // Pin turning forward
    uint8_t pinB;           // Pin turning backward
    uint8_t pinPWM;         // Pin enable/speed control

    uint8_t pwmChannel;     // Internal PWM channel

    uint8_t speed = 0;      // Speed of motor
};

#endif