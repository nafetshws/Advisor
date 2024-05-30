#ifndef TOF_HPP
#define TOF_HPP

#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <cstdint>

// I2C starting dddress of Sensors in hex
#define TOF_START_ADDR 0x30

// Error Offsets
#define TOF_LEFT_FRONT_OFFSET 8 
#define TOF_RIGHT_FRONT_OFFSET 23 
#define TOF_LEFT_OFFSET 10 
#define TOF_RIGHT_OFFSET 8 

class TOF {

    // sensor id
    uint8_t id;
    // I2C Address of the ToF Sensor
    uint8_t address;
    // Distance mesured in mm
    uint16_t distance;
    // Pin for enabling / disabling the i2c sensor
    uint8_t shutdownPin;
    // Sensor object of adafruit library
    Adafruit_VL53L0X sensor_obj = Adafruit_VL53L0X();
    // Mesure object to store the measurment
    VL53L0X_RangingMeasurementData_t measure;

public:

    /**
     * @brief   Create a new tof sensor object for measuring distance
    */
    TOF ();

    /**
     * @brief   Create a new tof sensor object for measuring distance
     * @param   id          unique sensor number
     * @param   address     i2c address of tof sensor
     * @param   shutdownPin pin for enable/disable sensor
    */
    TOF (uint8_t id, uint8_t address, uint8_t shutdownPin);


    /**
     * @brief   Enalbe or disable the tof sensor via the shutdwon pin
     *          true -> enables the sensor
     *          false -> disables the sensor
     * @param   enable  enable or disable
    */
    void enable(bool enable);


    /**
     * @brief   Start I2C communication with the sensor. Must be called in setup
     * @return  ture if init successful, false means cound't connect to sensor
    */
    bool begin();

    /**
     * @brief   Measures the distance and returns distance in mm, takes 10ms
     *          If stored value from the last measure should be use set
     *          parameter useStoredDist to true (takes 0ms instead of 10ms)
     * @param   useStoredDist   ture if distance from last measure should be returned
     * @return  Distance in mm
    */
    uint16_t getDist(bool useStoredDist = false);

};


/**
 * @brief   Starts the communiction with 2 tof sensor and sets
 *          the i2c address to the given value
 * @param   tof1 reference tof sensor 1
 * @param   tof2 reference tof sensor 2
*/
void initTofSensors(TOF &tof1, TOF &tof2);


/**
 * @brief   Starts the communiction with 4 tof sensor and sets
 *          the i2c address to the given value
 * @param   tof1 reference tof sensor 1
 * @param   tof2 reference tof sensor 2
 * @param   tof3 reference tof sensor 2
 * @param   tof4 reference tof sensor 2
*/
void initTofSensors(TOF &tof1, TOF &tof2, TOF &tof3, TOF &tof4);

#endif