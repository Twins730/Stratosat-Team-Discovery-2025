// code for stratosat LakeBurst
#include <SHC_BNO055.h>
#include <SHC_BME280.h>
#include <SHC_M9N.h>
#include <TimeLib.h>

int startup = now();
int i = 0;

int states = 0; // 0 = launch; 1 = liftoff; 2 = stabilization; 3 = burst; 4 = descent; 5 = landed;

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

  // print csv string for assembling the data to log. Needs a loop number and a state.
  Serial1.println(dataString(i, states));
  stateSwitcher();

  // now for the actual code based on state.
  switch (states) {
    case 0:
      // launch code. should just turn on status lights.
      break;
    case 1:
      // liftoff code
      break;
    case 2:
      // stablization code
      stabilize();
      break;
    case 3:
      // burst code
      break;
    case 4:
      // descent code
      break;
    case 5: 
      // landed code.
      break;
  }

}



String dataString(int a, int state) {
  // prefetch calls the current data
  bnowo.prefetchData();
  bmeup.prefetchData();
  miners.prefetchData();

  // return all data as a string
  return String(String("LAKEBURST") + String(",") + String(now() - startup) + String(",") + String(now()) + String(",") + String(a) 
    + String(",") + String(state) + String(",") + String(bmeup.getPressure()) + String(",") + 
    String(bmeup.getAltitude()) + String(",") + String(bmeup.getTemperature()) + String(",") + 
    String(bmeup.getHumidity()) + String(",") + String(bnowo.getAccelerationX()) + String(",") + String(bnowo.getAccelerationY()) 
    + String(",") + String(bnowo.getAccelerationZ()) + String(",") + String(bnowo.getGyroX()) + String(",") + 
    String(bnowo.getGyroY()) + String(",") + String(bnowo.getGyroZ()) + String(",") + String(miners.getLatitude()) + String(",") +
    String(miners.getLongitude()) + String(",") + String(miners.getAltitude()));
}

void stateSwitcher() {
  /* if( check for liftoff) {
    state = 1;
  }

  ... continue for all states
  */
}

void stabilize() {
  // this is where the stabilization code goes
}