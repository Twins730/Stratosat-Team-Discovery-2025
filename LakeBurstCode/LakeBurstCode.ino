// code for stratosat LakeBurst
#include <Adafruit_INA260.h>

#include <SHC_BNO055.h>
#include <SHC_BME280.h>
#include <SHC_M9N.h>
#include <TimeLib.h>

const int status = 21; // status light pin number
const int clockwise = 22; // clockwise pin number
const int cclockwise = 23; // counter clockwise pin number
const int LED = 21; // LED 

// See https://learn.adafruit.com/adafruit-ina260-current-voltage-power-sensor-breakout/arduino
// Note: This isnt in the Luma libraries
Adafruit_INA260 ina260 = Adafruit_INA260();

BNO055 bnowo; // create bno object pronounced "bean-owo"
SHC_BME280 bmeup; // create bme object pronounced "beamme-up" (ideally suffixed with Scotty)
M9N miners; // create a m9n object pronounced "minors"

int lasttime = millis();
int ii = 1; 
 

// get startup time
int startup = miners.getSecond();
int i = 0;

enum States {
  LAUNCH,
  LIFTOFF,
  STABILIZE,
  BURST,
  DESCENT,
  LANDED
};

States state = LAUNCH;


float lastAltitude = 0;
float peakAltitude = 0;
int velocityTime = 1;
float velocityEND = 0; 

void setup() {

 // Lights  
  pinMode(LED,OUTPUT);
  pinMode(clockwise,OUTPUT);
  pinMode(cclockwise,OUTPUT);
  pinMode(status,OUTPUT);


  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial1.begin(9600);

  
  Serial.println("Start blinking your pins twin");
  // Making Light THROB 
  digitalWrite(clockwise,HIGH);
  delay(500);
  digitalWrite(clockwise, LOW); 


  digitalWrite(cclockwise, HIGH);
  delay(500); 
  digitalWrite(cclockwise, LOW); 
  

  Serial.println("Start starting up yo sensors twin");
  // Start the sensors
  Serial.println("Startup");


  Serial.println(bnowo.init());
  //Serial.println(bmeup.init());
  Serial.println(miners.init());
  ina260.begin();
  

  

  // done
  Serial.println("initialization done.");
  
  
 
  

}

void loop() {
  // iterate loop by one each time
  ii++;

  // print csv string for assembling the data to log. Needs a loop number.
  printData();

  // now for the actual code based on state.
  switch (state) {
    case LAUNCH:
      // launch code. should just turn on status lights.
      digitalWrite(status, HIGH);
      break;
    case LIFTOFF:
      // liftoff code
      break;
    case STABILIZE:
      // stablization code
      stabilize();
      break;
    case BURST:
      // burst code
      burst();
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

  // Start of LED_Blink 
  if(millis() - lasttime <= 50){    // Light will be ON for 50 milisec
    digitalWrite(LED,HIGH); 
  }else if(millis() - lasttime < 1000){   // Light will turn OFF for remainder of 950 milisec
    digitalWrite(LED, LOW); 
  }else{
    lasttime = millis(); // Process with code
  }

  //stateSwitcher();

}


void printData() {
  // prefetch calls the current data
  bnowo.prefetchData();
  //bmeup.prefetchData();
  miners.prefetchData();

  Serial.println("Huh");

  
  // return all data as a single string
  Serial1.print(String("LAKEBURST") + String(","));
  Serial.println("huhx2");
  
  // Append the current mechine state.
  Serial1.print(String(state) + String(","));
  Serial.println("Huhx3");
  
  // Power sensor
  Serial1.print(String(ina260.readCurrent()) + String(","));
  Serial1.print(String(ina260.readBusVoltage()) + String(","));
  Serial1.print(String(ina260.readPower()) + String(","));

  /*
  // Append "bmeup" statistics
  Serial1.print(String(bmeup.getPressure()) + String(","));
  Serial1.print(String(bmeup.getAltitude()) + String(","));
  Serial1.print(String(bmeup.getTemperature()) + String(","));
  Serial1.print(String(bmeup.getHumidity()) + String(","));
  */
  
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
  
  // Append the 3d coordinates.
  Serial1.print(String(miners.getLatitude()) + String(","));
  Serial1.print(String(miners.getLongitude()) + String(","));
  Serial1.print(String(miners.getAltitude()));

  Serial1.println("");
}

void stateSwitcher() {
  
  /* if( check for liftoff) {
    state = 1;
  }

  ... continue for all states
  */

  // Get the current velocity
  int currentVelocity = velocity();
 
  // info from mrr slide
  

  // Check for liftoff altidude (if over 20m in the air is should be high enough to count)
  if(lastAltitude > 20 && state != LIFTOFF){
    state = LIFTOFF;
  }

  if (state == LIFTOFF) {
    // Check for the stabilize height
    // note: lastAltitude is mesured in meters.
    if (lastAltitude >= (20 * 1000)) {
      state = STABILIZE;
    }
  } 

  else if (state == STABILIZE) {
    if (peakAltitude > lastAltitude) {
      state = BURST;
    }
  }

  // Check if the payload continued to fall
  else if(state == BURST) {
    if (lastAltitude < (peakAltitude - 100)) {
      state = DESCENT;
    }
  }

  else if (state == DESCENT) {
    if(currentVelocity < 0.01) {
      // Check if the payloads velocity is very close to zero
      // Note: this accounts for innacuracies.
      state = LANDED;
    }
  }


  
}

// this is where the liftoff code goes
void liftoff(){
    
}

// this is where the burst code goes
void burst() {
    
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
    float p = 2;
    float d = 0.5 - bnowo.getGyroZ();

    float x = bnowo.getOrientationZ();
    float y = bnowo.getGyroZ();

    float upper_bound = 0;

    // Calculate the angle of the upper bounds
    if(x < a || x > -a) {
        upper_bound = -(p * x) + d;
    }
    // Calculate the left line of the upper bounds
    if(x <= -a) {
        upper_bound = (p * a) + d;
    }
    // Calculate the right line of the upper bounds
    if(x >= a){
        upper_bound = -(p * a) + d;
    }

    // Return 1 if the point is "above bounds"
    if(y <= upper_bound){
        return 1;
    }

    float lower_bound = 0;

    // Calculate the angle of the lower bounds
    if(x < a || x > -a){
        lower_bound = -(p * x) - d;
    }

    // Calculate the left line of the lower bounds
    if(x <= -a){
        lower_bound = (p * a) - d;
    }
    // Calculate the right line of the lower bounds
    if(x >= a){
        lower_bound = -(p * a) - d;
    }

    // Return negative 1 if the point is below the bounds
    if (y <= lower_bound){
        return -1;
    }

    // If this point is reached then none of the checks passed and the point is inside the bounds
    return 0;
}

// this is where the stabilization code goes
void stabilize() {
  
  // phase control
  if (phaseControl() == 1) {
    // turn on clockwise and off counter clockwise
    digitalWrite(clockwise, HIGH);
    digitalWrite(cclockwise, LOW);
  } else if (phaseControl() == -1) {
    // turn off all
    digitalWrite(clockwise, LOW);
    digitalWrite(cclockwise, LOW);
  } else {
    // turn on counter clockwise and off clockwise
    digitalWrite(cclockwise, HIGH);
    digitalWrite(clockwise, LOW);
  }
}

float velocity() {
 
  if (velocityTime >= 1000) {
    float DAltitude = bmeup.getAltitude() - lastAltitude;
    float DTime = millis() - velocityTime;
    DTime /= 1000;

    if (bmeup.getAltitude() >= lastAltitude) {
      peakAltitude = lastAltitude;
    }

    lastAltitude = bmeup.getAltitude();
    
    velocityTime = millis();

    //  Return vertical velocity in m/s
    velocityEND = (float)DAltitude / DTime;

    
  }
  return velocityEND; 
  
}
