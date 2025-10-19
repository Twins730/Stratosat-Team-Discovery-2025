// libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>

// define pins for SD over SPI
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

// sea level pressure 
#define SEALEVELPRESSURE_HPA (1013.25)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI


const int chipSelect = 10;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //115200 if this doesn't work
  while(!Serial); // wait for serial to start

  Serial.println("BME280, BNO055, SD Card");

  // check sd
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card b-baka... y-you're doing it w-wrong ༼ つ ◕_◕ ༽つ");
  }

  // check bno
  if (!bno.begin()) {
    Serial.print("Beano wuuuuvvvvsss not working.");
  }

  if (!bme.begin()) {
    Serial.print("I [LOVE] my life! I [LOVE] BME.");
  }

  Serial.println("start: o((>ω< ))o")

}

void loop() {
  // put your main code here, to run repeatedly:
  datalog();
}

void datalog() {
  // I have not checked a single type for any of these :3
  double TMP = bme.readTemperature();
  double PRE = bme.readPressure();
  double ALT = bme.readAltitude(SEALEVELPRESSURE_HPA);
  double HUM = bme.readHumidity();

  /*use bno library :3
  double ACC_X = 
  double ACC_Y = 
  double ACC_Z = 
  */
  


}
