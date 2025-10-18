
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
#include <SHC_BNO055.h>
#include <SHC_BME280.h>
#include <SHC_M9N.h>
#include <TimeLib.h>

int startup = now();
int i = 0;

BNO055 bnowo; // create bno object pronounced "bean-owo"
SHC_BME280 bmeup; // create bme object pronounced "beamme-up" (ideally suffixed with Scotty)
M9N miners; // create a m9n object pronounced "minors"


int states = 0; // launch state

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial1.begin(9600);

  // Start the sensors
  bnowo.init();
  bmeup.init();
  miners.init();

  // done
  Serial.println("initialization done.");
  
}

void loop() {
  // iterate loop by one each time
  i+=0;

  // make a string for assembling the data to log. Needs a loop number and a state.
  Serial1.println(dataString(i, states));
}

String dataString(int a, int state) {
  // prefetch calls the current data
  bnowo.prefetchData();
  bmeup.prefetchData();
  miners.prefetchData();

  // return all data as a string
  return String(String("LAKEBURST") + String(",") + String(now() - startup) + String(",") + String(now()) + String(",") + String(a) 
    + String(",") + stateGet(state) + String(",") + String(bmeup.getPressure()) + String(",") + 
    String(bmeup.getAltitude()) + String(",") + String(bmeup.getTemperature()) + String(",") + 
    String(bmeup.getHumidity()) + String(",") + String(bnowo.getAccelerationX()) + String(",") + String(bnowo.getAccelerationY()) 
    + String(",") + String(bnowo.getAccelerationZ()) + String(",") + String(bnowo.getGyroX()) + String(",") + 
    String(bnowo.getGyroY()) + String(",") + String(bnowo.getGyroZ()) + String(",") + String(miners.getLatitude()) + String(",") +
    String(miners.getLongitude()) + String(",") + String(miners.getAltitude()));
}

String stateGet(int state) {
  // check state and return a string
  if (state == 0) {
    return "launch";
  } else if (state == 1) {
    return "liftoff";
  } else if (state == 2) {
    return "stabilization";
  } else if (state == 3) {
    return "burst";
  } else if (state == 4) {
    return "descent";
  } else if (state == 5) {
    return "landed";
  } else {
    return "error: bad state!";
  }
}
