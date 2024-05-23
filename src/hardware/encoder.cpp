#include "../../include/encoder.hpp"


void Timer0_ISR() {
    // isr is called every 100 milli seconds
    // calc the revolutions per second
    //
    //  rps = revolution_per_100ms * 1s / 100ms
    //
    rpsA = MOTA_ROT_COUNT * INTERUPTS_PER_SECOND;
    rpsB = MOTB_ROT_COUNT * INTERUPTS_PER_SECOND;

    // set the counters to 0 for next calculation
    MOTA_ROT_COUNT = 0;
    MOTB_ROT_COUNT = 0;
}


void IRAM_ATTR EncoderA01_ISR() {
    // increase rotation count
    MOTA_ROT_COUNT++;
    // check rotation direction every 8 turns (bit mask 00..00111)
    if ( MOTA_ROT_COUNT & 0x7 == 0x7) {
        dirA = (digitalRead(pinEncoderA2)) ? 1 : -1;
    }
}


void IRAM_ATTR EncoderB01_ISR() {
    // increase rotation count
    MOTB_ROT_COUNT++;
    // check rotation direction every 8 turns (bit mask 00..00111)
    if ( MOTB_ROT_COUNT & 0x7 == 0x7) {
        dirB = (digitalRead(pinEncoderB2)) ? 1 : -1;
    }
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
    Timer0_Cfg = timerBegin(0, TIMER0_PRESCALAR, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, TIMER0_ALARM_AT, true);
    timerAlarmEnable(Timer0_Cfg);
}

uint16_t getRpsMotorA() {
    return rpsA;
}

uint16_t getRpsMotorB() {
    return rpsB;
}

int16_t getDirMotorA () {
    return rpsA * dirA;
}

int16_t getDirMotorB () {
    return rpsB * dirB;
}