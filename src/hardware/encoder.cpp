#include "../../include/encoder.hpp"


/* void Timer0_ISR() {
    // isr is called every 100 milli seconds
    // calc the revolutions per second of the wheel
    //
    //  rps = revolution_per_100ms * 1s / 100ms
    //
    rpsA = (ENC_RIGHT_COUNT / 3) * INTERUPTS_PER_SECOND;
    rpsB = (ENC_LEFT_COUNT / 3) * INTERUPTS_PER_SECOND;

    // set the counters to 0 for next calculation
    ENC_RIGHT_COUNT = 0;
    ENC_LEFT_COUNT = 0;
} */


void IRAM_ATTR EncoderA01_ISR() {
    // increase rotation count
    ENC_RIGHT_COUNT++;
}


void IRAM_ATTR EncoderB01_ISR() {
    // increase rotation count
    ENC_LEFT_COUNT++;
}

void initEncoders(uint8_t encA01, uint8_t encA02, uint8_t encB01, uint8_t encB02) {

    // set the encoder pins as input pullup, for the digital encoder signal
    pinMode(encA01, INPUT_PULLUP);
    pinMode(encA02, INPUT_PULLUP);
    pinMode(encB01, INPUT_PULLUP);
    pinMode(encB02, INPUT_PULLUP);

    // stores the encoder2 pins to determand the direction
    pinEncoderA2 = encA02;
    pinEncoderB2 = encB02;

    // delcare a1 and b1 as interrupt pins and attach the isr
    attachInterrupt(encA01, &EncoderA01_ISR, RISING);
    attachInterrupt(encB01, &EncoderB01_ISR, RISING);

    // configure the timer0 interrupt
    // Timer0_Cfg = timerBegin(0, TIMER0_PRESCALAR, true);
    // timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    // timerAlarmWrite(Timer0_Cfg, TIMER0_ALARM_AT, true);
    // timerAlarmEnable(Timer0_Cfg);
}

/* uint32_t getRpsMotorA() {
    return rpsA;
}

uint32_t getRpsMotorB() {
    return rpsB;
}

int16_t getDirMotorA () {
    return rpsA * dirA;
}

int16_t getDirMotorB () {
    return rpsB * dirB;
} */

uint32_t getEncRight() {
    return ENC_RIGHT_COUNT;
}

uint32_t getEncLeft() {
    return ENC_LEFT_COUNT;
}

void resetLeftEncoder() {
    ENC_LEFT_COUNT = 0;
}

void resetRightEncoder() {
    ENC_RIGHT_COUNT = 0;
}