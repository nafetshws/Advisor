#ifndef IR_HPP
#define IR_HPP

#include <cstdint>

class IR {
    public:
        // Output pin (digital) of sensor
        uint8_t IR_SENSOR_PIN;
 
        /**
         * @brief   Create new IR sensor object 
         * @param   IR_SENSOR_PIN   Digital output pin to read data   
        */
        IR (uint8_t IR_SENSOR_PIN);

        /**
         * @brief   Measure if the distance is small enough to trigger
         *          the sensor
         * @return  true if the sensor is close, false if nothing is detected
        */
        bool isTriggered();
};

#endif