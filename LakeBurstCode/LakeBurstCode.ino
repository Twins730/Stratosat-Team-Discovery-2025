// code for stratosat LakeBurst
#include <SHC_BNO055.h>
#include <SHC_BME280.h>
#include <SHC_M9N.h>
#include <TimeLib.h>

const int status = 12; // status light pin number
const int clockwise = 20;
const int cclockwise = 21; // counter clockwise pin number
const int led = 4; // LED 

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
}

void loop() {
  // iterate loop by one each time
  i+=0;

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

// this is where the descent code goes
void descent(){
    
}

// this is where the landed code goes
void landed() {
    
}

// Returns 1 if the point is above the channel and negative one if it is below. Otherwise it returns zero
int phaseControl(){
    // define constants
    int a = 100
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