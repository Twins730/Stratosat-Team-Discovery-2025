//###############################################################################//
//#█      ███  █  █  █████ ████  █   █ ████  █████ █████  ████  ███  ████  █████#// 
//#█     █   █ █ █   █     █   █ █   █ █   █ █       █   █     █   █ █   █ █    #//
//#█     █████ ██ █  ████  ████  █   █ ████  █████   █   █     █   █ █   █ ████ #// 
//#█     █   █ █  █  █     █   █ █   █ █   █     █   █   █     █   █ █   █ █    #//
//#█████ █   █ █   █ █████ ████  █████ █   █ █████   █    ████  ███  ████  █████#//
//###############################################################################//
// code for stratosat LakeBurst
#include <Adafruit_INA260.h>
#include <SHC_BNO055.h>
#include <Bme280.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h> //Use Library Manager or download here: https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
#include <TinyGPS++.h>
#include <TimeLib.h>

// See https://learn.adafruit.com/adafruit-ina260-current-voltage-power-sensor-breakout/arduino
// Note: This isnt in the Luma libraries
Adafruit_INA260 ina260 = Adafruit_INA260();
BNO055 bnowo; // create bno object pronounced "bean-owo"
I2CGPS geepers; // jeepers?
TinyGPSPlus gps;
Bme280TwoWire bmeup; // create bme object pronounced "beamme-up" (ideally suffixed with Scotty)

// Pinout numbers
const int status = 21; // status light pin number
const int clockwise = 22; // clockwise pin number
const int cclockwise = 23; // counter clockwise pin number
const int LED = 21; // LED 

// State change variables.
float lastAltitude = 0;
float peakAltitude = 0;
int velocityTime = 1;
float velocityEND = 0; 
float currentVelocity = 0;

// Timing variables.
int lasttime = 0;
int ii = 1; 
 
// get startup time
int startup = 1;
int i = 0;

// This enum handles the mode or states that the payload can be in.
enum States {
  LAUNCH,
  LIFTOFF,
  STABILIZE,
  DESCENT,
  LANDED
};

// Initialize the current state to launch
States state = LAUNCH;

// The setup function kindly provided to us by Arduino.
void setup() {
  Wire.begin();

  // Lights  
  pinMode(LED,OUTPUT);
  pinMode(clockwise,OUTPUT);
  pinMode(cclockwise,OUTPUT);
  pinMode(status,OUTPUT);

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial1.begin(9600);
  
  // Make clockwise Light THROB 
  digitalWrite(clockwise,HIGH);
  delay(50);
  digitalWrite(clockwise, LOW); 

  // Make counter clockwise Light TROB
  digitalWrite(cclockwise, HIGH);
  delay(50); 
  digitalWrite(cclockwise, LOW); 

  // Start the sensors
  Serial.println("Startup");

  // Announce that the bnowo has initialized to the serial monitor
  Serial.println(bnowo.init());

  // Initialize the other components
  geepers.begin();
  ina260.begin();
  bmeup.begin(Bme280TwoWireAddress::Primary);
  bmeup.setSettings(Bme280Settings::indoor());

  // Announce that the initializeation is done
  Serial.println("initialization done.");
}

void loop() {
  // iterate loop by one each time
  ii++;

  // commenting out for testing the stabilization
  // Start of LED_Blink 

  // Read the bytes from the geepers to the gps.
  while (geepers.available()) { //available() returns the number of new bytes available from the GPS module
    gps.encode(geepers.read()); //Feed the GPS parser
  }
  
  // Handle status lights
  if(millis() - lasttime <= 200) {    // Light will be ON for 500 milisec
    digitalWrite(LED,HIGH); 
  } else if(millis() - lasttime < 1000) {   // Light will turn OFF for remainder of 500 milisec
    digitalWrite(LED, LOW); 
  }else {
    lasttime = millis(); // Process with code
  }
  
  // prefetch calls the current data
  bnowo.prefetchData();
  
  // Send the altitude to the serial monitor
  Serial.println(altitude());
  
  // print csv string for assembling the data to log. Needs a loop number.
  printData();

  // now for the actual code based on state.
  switch (state) {
    case LAUNCH:
      // launch code. should just turn on status lights.
      break;
    case LIFTOFF:
      // liftoff code
      break;
    case STABILIZE:
      // stablization code
      stabilize();
      break;
    case DESCENT:
      // descent code
      descent();
      break;
    case LANDED: 
      // landed code.
      landed();
      break;
  }
  stateSwitcher();
}

// Save the data to the sd card through serial1.
// note: this saves the data as a log<number>.txt
//    this needs to be converted to a csv file 
//    starting with the header on the next line
// ####CSV HEADER:"Team name,Time,Current State,INA Current,INA Voltage,INA Power,BMEUP Pressure,BMEUP Tempurature,BMEUP Humidity,BNOWO Acceleration X,BNOWO Acceleration Y,BNOWO Acceleration Z,BNOWO Gyro X,BNOWO Gyro Y,BNOWO Gyro Z,BNOWO Orientation X,BNOWO Orientation Y,BNOWO Orientation Z,Altitude"####
void printData() {
  // return all data as a single string
  Serial1.print(String("LAKEBURST") + String(","));
  
  Serial1.print(String(millis() + String(",")));

  // Append the current mechine state.
  Serial1.print(String(state) + String(","));
  
  // Power sensor
  Serial1.print(String(ina260.readCurrent()) + String(","));
  Serial1.print(String(ina260.readBusVoltage()) + String(","));
  Serial1.print(String(ina260.readPower()) + String(","));

  // Append "bmeup" statistics
  Serial1.print(String(bmeup.getPressure() / 100.0) + String(","));
  Serial1.print(String(bmeup.getTemperature()) + String(","));
  Serial1.print(String(bmeup.getHumidity()) + String(","));
  
  // Append Acceleration.
  Serial1.print(String(bnowo.getAccelerationX()) + String(",")); 
  Serial1.print(String(bnowo.getAccelerationY()) + String(","));
  Serial1.print(String(bnowo.getAccelerationZ()) + String(","));
  
  // Append Gyro Axis.
  Serial1.print(String(bnowo.getGyroX()) + String(","));
  Serial1.print(String(bnowo.getGyroY()) + String(","));
  Serial1.print(String(bnowo.getGyroZ()) + String(","));
  
  // Append the Orientation.
  Serial1.print(String(bnowo.getOrientationX()) + String(",")); 
  Serial1.print(String(bnowo.getOrientationY()) + String(",")); 
  Serial1.print(String(bnowo.getOrientationZ()) + String(","));
  
  if (gps.altitude.isValid())
  {
    Serial1.print(F("Altitude Meters:"));
    Serial1.print(gps.altitude.meters());
  }
  else {Serial1.print(F("ÄLTITUDE_SAD"));}

  Serial1.println("");
  Serial.println("write done");
}

void stateSwitcher() {
  // Calculate the altitude.
  lastAltitude = altitude();

  // Get the current velocity
  currentVelocity = velocity();
 
  // info from mrr slide
  // Check for liftoff altidude (if over 20m in the air is should be high enough to count)
  if (state == LAUNCH){
    if (lastAltitude >= 500){
      state = LIFTOFF;
    }
  }

  if (state == LIFTOFF) {
    // Check for the stabilize height
    // note: lastAltitude is mesured in meters.
    if (lastAltitude >= (18 * 1000)) {
      state = STABILIZE;
    }
  } else if (state == STABILIZE) {
    if (peakAltitude-200 > lastAltitude) {
      state = DESCENT;
    }
  } else if (state == DESCENT) {
    if(currentVelocity*currentVelocity < 0.1 && lastAltitude <= 100) {
      // Check if the payloads velocity is very close to zero
      // Note: this accounts for innacuracies.
      state = LANDED;
    }
  }
}

// this is where the liftoff code goes
void liftoff(){
    
}


// this is where the descent code goes
void descent(){
    
}

// this is where the landed code goes
void landed() {
    
}

// Returns 1 if the point is above the channel and negative one if it is below. Otherwise it returns zero
int phaseControl(){
    // define constants
    int a = 100;
    float p = .4;
    float d = 15;

    // Get the orientation and velocity as X and Y for the function inputs.
    float x = bnowo.getOrientationX();
    float y = bnowo.getGyroZ();
    // Send these values to the serial monitor
    Serial.println(String(x) + String(",") + String(y));

    if ((x <= a+180) && (x >= -a+180)) {
      if ((p*(x-180) + y) >= d) {
        return(-1);
      }
      else if ((p*(x-180) + y <= -d)) {
        return(1);
      }
      else {return(0);}
    }
    else if ((x<=-a+180)) {
      if ((y >= p*a + d)) {
        return(-1);
      } else if ((y <= p*a - d)) {
        return(1);
      }
      else {return(0);}

    }
    else if ((x>=a+180)) {
      if ((y >= (-p * a + d))) {
        return(-1);
      } else if ((y <= (-p * a - d))) {
        return(1);
      }
      else {return(0);}
    }
    else {return(0);}
}

// this is where the stabilization code goes
void stabilize() {
  Serial.println("Stabilize");
  int temp = phaseControl();
  Serial.println(temp);
  // phase control
  if (temp == 1) {
    // turn on clockwise and off counter clockwise
    digitalWrite(clockwise, HIGH);
    digitalWrite(cclockwise, LOW);
    Serial.println("clockwise");
  } else if (temp == -1) {
    // turn on counter clockwise and off clockwise
    digitalWrite(cclockwise, HIGH);
    digitalWrite(clockwise, LOW);
    Serial.println("cclockwise");

  } else {
    // turn off all
    digitalWrite(clockwise, LOW);
    digitalWrite(cclockwise, LOW);
  }
}

// Calculate the current velocity.
float velocity() {
  
  // Get the current altiude
  float alt = altitude();
  
  // Compare the altitude with the last known altitude to get the velocity.
  if (velocityTime >= 1000) {
    float DAltitude = alt - lastAltitude;
    float DTime = millis() - velocityTime;
    DTime /= 1000;
    if (alt >= lastAltitude) {
      peakAltitude = lastAltitude;
    }
    lastAltitude = alt;
    velocityTime = millis();

    //  Return vertical velocity in m/s
    velocityEND = (float)DAltitude / DTime;
  }
  return velocityEND; 
  
}

// Pull the current altitude with a fallback option.
float altitude() {
  if (gps.altitude.isValid()) {
    return (gps.altitude.meters());
  }
  else {
  return (44330.0*(1.0 - pow(((float) bmeup.getPressure() / 100.0) / 1013.25, 0.1903)));
  }
}
