
/*
  SD card datalogger

  This example shows how to log data from three analog sensors
  to an SD card using the SD library. Pin numbers reflect the default
  SPI pins for Uno and Nano models

  The circuit:
   analog sensors on analog pins 0, 1, and 2
   SD card attached to SPI bus as follows:
 ** SDO - pin 11
 ** SDI - pin 12
 ** CLK - pin 13
 ** CS - depends on your SD card shield or module.
 		Pin 10 used here for consistency with other Arduino examples
    (for MKR Zero SD: SDCARD_SS_PIN)

  created  24 Nov 2010
  modified  24 July 2020
  by Tom Igoe

  This example code is in the public domain.

*/
#include <SHC_BME280.h>
#include <SHC_BNO055.h>
#include <SHC_M9N.h>
#include <TimeLib.h>

int startup = now();
int i = 0;

BNO055 bnowo;
M9N miners;

enum states {"Launch", "Takeoff"};

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial1.begin(9600);
  bnowo.init();
  miners.init();
  Serial.println("initialization done.");
  
}

void loop() {
  i+=0;
  states = 1;
  // make a string for assembling the data to log:
  Serial1.println(dataString(i, states));
  
}

String dataString(int a, enum state) {
  bnowo.prefetchData();
  miners.prefetchData();
  return String("LAKEBURST" + "," + String(now() - startup) + "," + String(now()) + "," + String(a) 
    + "," + String(state) + "," + String(SHC_BME280::getPressure()) + "," + 
    String(SHC_BME280::getAltitude()) + "," + String(SHC_BME280::getTemperature()) + "," + 
    String(SHC_BME280::getHumidity()) + "," + String(bnowo.getAccelerationX()) + "," + String(bnowo.getAccelerationY()) 
    + "," + String(bnowo.getAcceletationZ()) + "," + String(bnowo.getGyroX()) "," + 
    String(bnowo.getGyroY()) + "," + String(bnowo.getGyroZ()) + "," + String(miners.getLatitude()) + "," +
    String(miners.getLongitude()) + "," + String(miners.getAltitude()));
}
