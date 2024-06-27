#ifndef TOF_HPP
#define TOF_HPP

#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL6180X.h>
#include <Arduino.h>
#include <cstdint>

// I2C starting dddress of Sensors in hex
#define TOF_START_ADDR 0x40

// Error Offsets
// #define TOF_LEFT_FRONT_OFFSET 0 
// #define TOF_RIGHT_FRONT_OFFSET 3 
// #define TOF_LEFT_OFFSET 0 
// #define TOF_RIGHT_OFFSET 0 

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


class TOF_6180 {

    // sensor id
    uint8_t id;
    // I2C Address of the ToF Sensor
    uint8_t address;
    // Distance mesured in mm
    uint16_t distance;
    // Pin for enabling / disabling the i2c sensor
    uint8_t shutdownPin;
    // Sensor object of adafruit library
    Adafruit_VL6180X sensor_obj = Adafruit_VL6180X();

public:
    static int16_t TOF_LEFT_FRONT_OFFSET; 
    static int16_t TOF_RIGHT_FRONT_OFFSET;
    static int16_t TOF_LEFT_OFFSET; 
    static int16_t TOF_RIGHT_OFFSET; 

    /**
     * @brief   Create a new tof sensor object for measuring distance
    */
    TOF_6180 ();

    /**
     * @brief   Create a new tof sensor object for measuring distance
     * @param   id          unique sensor number
     * @param   address     i2c address of tof sensor
     * @param   shutdownPin pin for enable/disable sensor
    */
    TOF_6180 (uint8_t id, uint8_t address, uint8_t shutdownPin);


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
 * @brief   Starts the communiction with 4 tof sensor and sets
 *          the i2c address to the given value
 * @param   tof1 reference tof sensor 1
 * @param   tof2 reference tof sensor 2
 * @param   tof3 reference tof sensor 2
 * @param   tof4 reference tof sensor 2
*/
void initTofSensors(TOF &tof1, TOF &tof2, TOF &tof3, TOF &tof4);

/**
 * @brief   Starts the communiction with 4 VL6180 tof sensor and sets
 *          the i2c address to the given value
 * @param   tof1 reference tof sensor 1
 * @param   tof2 reference tof sensor 2
 * @param   tof3 reference tof sensor 2
 * @param   tof4 reference tof sensor 2
*/
void initTofSensors(TOF_6180 &tof1, TOF_6180 &tof2, TOF_6180 &tof3, TOF_6180 &tof4);

/**
 * @brief   Starts the communiction with 2 old tof sensors and 2 VL6180 tof sensor and sets
 *          the i2c address to the given value
 * @param   tof1 reference vl53L0 sensor 1
 * @param   tof2 reference vl53L0 sensor 2
 * @param   tof3 reference vl6180 sensor 1
 * @param   tof4 reference vl6180 sensor 2
*/
void initTofSensors(TOF &tof1, TOF &tof2, TOF_6180 &tof3, TOF_6180 &tof4);


/**
 * @brief   Prints all connected devices to the I2C Bus
 */
void i2CScanner();


#endif