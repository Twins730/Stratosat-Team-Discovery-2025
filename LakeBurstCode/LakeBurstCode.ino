// code for stratosat LakeBurst
#include <SHC_BNO055.h>
#include <SHC_BME280.h>
#include <SHC_M9N.h>
#include <TimeLib.h>


const int status = 12; // status light pin number
const int clockwise = 20;
const int cclockwise = 21; // counter clockwise pin number
const int LED = 12; // A NEEDS TO BE CHANGED TO PIN DESIGNAION!!!!! 
const int lasttime = SHC_M9N.h;


int lasttime = millis();

// get startup time
int startup = now();
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

BNO055 bnowo; // create bno object pronounced "bean-owo"
SHC_BME280 bmeup; // create bme object pronounced "beamme-up" (ideally suffixed with Scotty)
M9N miners; // create a m9n object pronounced "minors"

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
  
  // Lights  
  pinMode(LED,output);

}

void loop() {
  // iterate loop by one each time
  i+=0;

  // print csv string for assembling the data to log. Needs a loop number and a state.
  Serial1.println(dataString(i));

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
      decent();
      break;
    case LANDED: 
      // landed code.
      landed();
      break;
  }

void loop(){

  // Start of LED_Blink 
 


  if(millis() - lasttime <= 50){    // Light will be ON for 50 milisec
    digitalWrite(LED,HIGH); 
  }else if(millis() - lasttime < 1000){   // Light will turn OFF for remainder of 950 milisec
    digitalWrite(LED, LOW); 
  }else{
    lasttime = millis(); // Process with code
  }


}




  }

  digitalWrite(status, LOW);
  stateSwitcher();

}


String dataString(int a) {
  // prefetch calls the current data
  bnowo.prefetchData();
  bmeup.prefetchData();
  miners.prefetchData();
  
  // return all data as a single string
  return String(String("LAKEBURST") + "," + 
      // Append timing
      String(now() - startup) + "," + 
      String(now()) + "," + 
      
      // Append "a" variable
      String(a) + "," + 
      
      // Append the current mechine state.
      String(state) + "," + 
      
      // Append "bmeup" statistics
      String(bmeup.getPressure()) + "," + 
      String(bmeup.getAltitude()) + "," +
      String(bmeup.getTemperature()) + "," + 
      String(bmeup.getHumidity()) + "," + 
      
      // Append Acceleration.
      String(bnowo.getAccelerationX()) + "," + 
      String(bnowo.getAccelerationY()) + "," + 
      String(bnowo.getAccelerationZ()) + "," + 
      
      // Append Gyro Axis.
      String(bnowo.getGyroX()) + "," + 
      String(bnowo.getGyroY()) + "," + 
      String(bnowo.getGyroZ()) + "," + 
      
      // Append the Orientation.
      String(bnowo.getOrientationX()) + "," + 
      String(bnowo.getOrientationY()) + "," + 
      String(bnowo.getOrientationZ()) + "," + 
      
      // Append the 3d coordinates.
      String(miners.getLatitude()) + "," +
      String(miners.getLongitude()) + "," + 
      String(miners.getAltitude()));
}

void stateSwitcher() {
  /* if( check for liftoff) {
    state = 1;
  }

  ... continue for all states
  */
}

// this is where the liftoff code goes
void liftoff(){
  


}

// this is where the burst code goes
void burst() {
    
}

// this is where the decent code goes
void decent(){
    
}

// this is where the landed code goes
void landed() {
    
}


// this is where the stabilization code goes
void stabilize() {
  // define constants (slope )
  float k_p = 1.0;
  float k_v = 1.0;
  float deadzone = 15;

  // phase control
  if (k_p * bnowo.getOrientationX() + k_v * bnowo.getGyroX() <= -deadzone) {
    // turn on clockwise and off counter clockwise
    digitalWrite(clockwise, HIGH);
    digitalWrite(cclockwise, LOW);
  } else if (k_p * bnowo.getOrientationX() + k_v * bnowo.getGyroX() <= deadzone) {
    // turn on counter clockwise and off clockwise
    digitalWrite(cclockwise, HIGH);
    digitalWrite(clockwise, LOW);
  } else {
    // turn off all
    digitalWrite(clockwise, LOW);
    digitalWrite(cclockwise, LOW);
  }
}
