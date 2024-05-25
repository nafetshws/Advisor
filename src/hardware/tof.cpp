#include "../../include/tof.hpp"


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

  // return the distance in mm
  distance = measure.RangeMilliMeter;
  return distance;
}


void initTofSensors(TOF &tof1, TOF &tof2) {

  // reset all sensor
  tof1.enable(false);
  tof2.enable(false);
  delay(10);

  // unreset all sensors
  tof1.enable(true);
  tof2.enable(true);
  delay(10);

  // reset all sensors again
  tof1.enable(false);
  tof2.enable(false);
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
}

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

