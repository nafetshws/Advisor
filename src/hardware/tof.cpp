#include "../../include/tof.hpp"


// TOF VL53L0X //////////////////////////////////

TOF::TOF() {}


TOF::TOF(uint8_t id, uint8_t address, uint8_t shutdownPin) {
  this->id = id;
  this->address = address;
  this->shutdownPin = shutdownPin;

  pinMode(shutdownPin, OUTPUT);
}


void TOF::enable(bool enable) {
  digitalWrite(shutdownPin, enable);
}


bool TOF::begin() {
  // initing the i2c communication with the right address
  if(!sensor_obj.begin(address)) {
    // Communication faild to establish
    Serial.printf("ERROR: Faild to set address on tof sensor %d, PLEASE REBOOT", id);
    // return error code
    return 0;
  }
  // success, return code 1
  return 1;
}


uint16_t TOF::getDist(bool useStoredDist) {

  if (useStoredDist) return this->distance;

  // start the measurement with the sensor object and save into measure object
  sensor_obj.rangingTest(&measure, false);

  // If measurment went wrong
  if (measure.RangeStatus == 4) return -1;
  // Do out of Range check and set dist to max of 8000
  if (measure.RangeMilliMeter > 60000) return 8000;
  // return the distance in mm
  distance = measure.RangeMilliMeter;

  // Subract physical error of tof sensor
  switch (this->id) {
    case 1:
      distance -= TOF_LEFT_FRONT_OFFSET;
      break;
    case 2:
      distance -= TOF_RIGHT_FRONT_OFFSET;
      break;
    case 3:
      distance -= TOF_LEFT_OFFSET;
      break;
    case 4:
      distance -= TOF_RIGHT_OFFSET;
      break;
    default:
      Serial.printf("Error!\n");
      break;
  }

  return distance;
}


// TOF VL6180X //////////////////////////////////

TOF_6180::TOF_6180() {}


TOF_6180::TOF_6180(uint8_t id, uint8_t address, uint8_t shutdownPin) {
  this->id = id;
  this->address = address;
  this->shutdownPin = shutdownPin;

  pinMode(shutdownPin, OUTPUT);
}


void TOF_6180::enable(bool enable) {
  digitalWrite(shutdownPin, enable);
}


bool TOF_6180::begin() {
  // initing the i2c communication with the right address
  if(!sensor_obj.begin()) {
    // Communication faild to establish
    Serial.printf("ERROR: Faild to init tof sensor %d, PLEASE REBOOT", id);
    // return error code
    return 0;
  }
  // set address
  sensor_obj.setAddress(this->address);
  delay(10);

  if (this->address != sensor_obj.getAddress()) {
    Serial.printf("ERROR: Faild to set address tof sensor %d, PLEASE REBOOT", id);
    return 0;
  }

  // success, return code 1
  return 1;
}


uint16_t TOF_6180::getDist(bool useStoredDist) {

  if (useStoredDist) return this->distance;

  // start the measurement with the sensor object
  uint16_t dist = sensor_obj.readRange();
  uint8_t status = sensor_obj.readRangeStatus();

  // if measurement fail, return error
  if (status != VL6180X_ERROR_NONE) return -1;

  // return the distance in mm
  this->distance = dist;

  // Subract physical error of tof sensor
  switch (this->id) {
    case 1:
      distance -= TOF_LEFT_FRONT_OFFSET;
      break;
    case 2:
      distance -= TOF_RIGHT_FRONT_OFFSET;
      break;
    case 3:
      distance -= TOF_LEFT_OFFSET;
      break;
    case 4:
      distance -= TOF_RIGHT_OFFSET;
      break;
    default:
      Serial.printf("ERROR: Offest couldn't be applied on tof\n");
      break;
  }

  return distance;
}


// Init Functions ///////////////////////////////

void initTofSensors(TOF &tof1, TOF &tof2, TOF &tof3, TOF &tof4) {

  // reset all sensor
  tof1.enable(false);
  tof2.enable(false);
  tof3.enable(false);
  tof4.enable(false);
  delay(10);

  // unreset all sensors
  tof1.enable(true);
  tof2.enable(true);
  tof3.enable(true);
  tof4.enable(true);
  delay(10);

  // reset all sensors again
  tof1.enable(false);
  tof2.enable(false);
  tof3.enable(false);
  tof4.enable(false);
  delay(10);
  
  // only enable one at a time to set the address
  
  // tof 1
  tof1.enable(true);
  tof1.begin();
  delay(10);

  // tof 2
  tof2.enable(true);
  tof2.begin();
  delay(10);

  // tof 3
  tof3.enable(true);
  tof3.begin();
  delay(10);

  // tof 4
  tof4.enable(true);
  tof4.begin();
  delay(10);
}


void initTofSensors(TOF_6180 &tof1, TOF_6180 &tof2, TOF_6180 &tof3, TOF_6180 &tof4) {

  // reset all sensor
  tof1.enable(false);
  tof2.enable(false);
  tof3.enable(false);
  tof4.enable(false);
  delay(10);

  // unreset all sensors
  tof1.enable(true);
  tof2.enable(true);
  tof3.enable(true);
  tof4.enable(true);
  delay(10);

  // reset all sensors again
  tof1.enable(false);
  tof2.enable(false);
  tof3.enable(false);
  tof4.enable(false);
  delay(10);
  
  // only enable one at a time to set the address
  
  // tof 1
  tof1.enable(true);
  tof1.begin();
  delay(10);

  // tof 2
  tof2.enable(true);
  tof2.begin();
  delay(10);

  // tof 3
  tof3.enable(true);
  tof3.begin();
  delay(10);

  // tof 4
  tof4.enable(true);
  tof4.begin();
  delay(10);
}



void initTofSensors(TOF &tof1, TOF &tof2, TOF_6180 &tof3, TOF_6180 &tof4) {

  // reset all sensor
  tof1.enable(false);
  tof2.enable(false);
  tof3.enable(false);
  tof4.enable(false);
  delay(10);

  // unreset all sensors
  tof1.enable(true);
  tof2.enable(true);
  tof3.enable(true);
  tof4.enable(true);
  delay(10);

  // reset all sensors again
  tof1.enable(false);
  tof2.enable(false);
  tof3.enable(false);
  tof4.enable(false);
  delay(10);
  
  // only enable one at a time to set the address
  
  // tof 1
  tof1.enable(true);
  tof1.begin();
  delay(10);

  // tof 2
  tof2.enable(true);
  tof2.begin();
  delay(10);

  // tof 3
  tof3.enable(true);
  tof3.begin();
  delay(10);

  // tof 4
  tof4.enable(true);
  tof4.begin();
  delay(10);
}

