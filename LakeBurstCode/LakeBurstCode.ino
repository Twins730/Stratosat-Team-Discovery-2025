// code for stratosat LakeBurst
#include <SHC_BNO055.h>
#include <SHC_BME280.h>
#include <SHC_M9N.h>
#include <TimeLib.h>

const int status = 21; // status light pin number
const int clockwise = 22; // clockwise pin number
const int cclockwise = 23; // counter clockwise pin number
const int LED = 21; // LED 

BNO055 bnowo; // create bno object pronounced "bean-owo"
SHC_BME280 bmeup; // create bme object pronounced "beamme-up" (ideally suffixed with Scotty)
M9N miners; // create a m9n object pronounced "minors"

int lasttime = millis();
 

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
  pinMode(LED,OUTPUT);
  pinMode(clockwise,OUTPUT);
  pinMode(cclockwise,OUTPUT);
  pinMode(status,OUTPUT);


}

void loop() {
  // iterate loop by one each time
  i++;

  // print csv string for assembling the data to log. Needs a loop number.
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

  digitalWrite(status, LOW);
  stateSwitcher();

}


String dataString(int a) {
  // prefetch calls the current data
  bnowo.prefetchData();
  bmeup.prefetchData();
  miners.prefetchData();
  
  // return all data as a single string
  return String(String("LAKEBURST") + String(",") + 
      // Append timing
      String(now() - startup) + String(",") + 
      String(now()) + String(",") + 
      
      // Append "a" variable
      String(a) + String(String(",")) + 
      
      // Append the current mechine state.
      String(state) + String(String(",")) + 
      
      // Append "bmeup" statistics
      String(bmeup.getPressure()) + String(",") + 
      String(bmeup.getAltitude()) + String(",") +
      String(bmeup.getTemperature()) + String(",") + 
      String(bmeup.getHumidity()) + String(",") + 
      
      // Append Acceleration.
      String(bnowo.getAccelerationX()) + String(",") + 
      String(bnowo.getAccelerationY()) + String(",") + 
      String(bnowo.getAccelerationZ()) + String(",") + 
      
      // Append Gyro Axis.
      String(bnowo.getGyroX()) + String(",") + 
      String(bnowo.getGyroY()) + String(",") + 
      String(bnowo.getGyroZ()) + String(",") + 
      
      // Append the Orientation.
      String(bnowo.getOrientationX()) + String(",") + 
      String(bnowo.getOrientationY()) + String(",") + 
      String(bnowo.getOrientationZ()) + String(",") + 
      
      // Append the 3d coordinates.
      String(miners.getLatitude()) + String(",") +
      String(miners.getLongitude()) + String(",") + 
      String(miners.getAltitude()));
}

void stateSwitcher() {
  
  /* if( check for liftoff) {
    state = 1;
  }

  ... continue for all states
  */

  // info from mrr slide
  

  // Check for liftoff altidude (if over 20m in the air is should be high enough to count)
  if(lastAltitude > 20 && state != LIFTOFF){
    state = LIFTOFF;
  }

  // Check for the stabilize height
  // note: lastAltitude is mesured in meters.
  else if(lastAltitude >= (20 * 1000) && state != STABILIZE){
    state = STABILIZE;
  }


  // Check if the baloon is descending
  else if(peakAltitude > lastAltitude && state != BURST) {
    state = BURST;
  }

  // Check if the payload continued to fall
  else if(lastAltitude < (peakAltitude - 100) && state != DESCENT){
    state = DESCENT;
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
    float p = 1.0;
    float d = 15;

    float x = bnowo.getOrientationX();
    float y = bnowo.getGyroX();

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
    // turn on counter clockwise and off clockwise
    digitalWrite(cclockwise, HIGH);
    digitalWrite(clockwise, LOW);
  } else {
    // turn off all
    digitalWrite(clockwise, LOW);
    digitalWrite(cclockwise, LOW);
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
