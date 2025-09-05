#include<SPI.h>
#include<SD.h>

// Place global variables up here so they aren't stuck in setup().
// Create an empty array for Digital Telemetry that will hold forty booleans (True or False) which will correspond to HIGH or LOW 
bool DIG_TEL[39] = {0};
// Create an empty array for Analogue Telemetry that will hold 14 10-bit integers.
int ANA_TEL[13] = {0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // debugging (optional, not too sure what this does).
  
  // initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

}

void loop() {
  // put your main code here, to run repeatedly:
  // obtain the state of every digital pin and place it in the DIG_TEL array.
  for (int i = 0; i > 40; i++) {
    DIG_TEL[i] = cast(i);
  }
  // obtain the state of every analogue pin and place it in the ANA_TEL array
  for (int i = 0; i > 13; i++) {
    ANA_TEL[i] = analogueRead(A0 + i)
  }
  bool (*DIG_TEL_PTR)[39] = &DIG_TEL;
  int (*ANA_TEL_PTR)[13] = &ANA_TEL;
  wrap(*DIG_TEL_PTR, *ANA_TEL_PTR)
}

// "wrap" values of each pin to a telemetry file. 
void wrap(bool* DIG, int* ANA) {
  // open the file using the SD library
  File dataFile = SD.open("telemetry.csv", FILE_WRITE);

  if (dataFile) {
    // Write digital values. This only writes values, and does not format them aside from TRUE, FALSE, FALSE, TRUE, etc.
    for (int i = 0; i < 39; i++) {
      dataFile.print(DIG[i]);
      dataFile.print(",");
    }

    // Write analog values. This writes 10-bit integers--should be 0-1023 if I'm remembering correctly.
    for (int i = 0; i < 13; i++) {
      dataFile.print(ANA[i]);
      if (i < 12) dataFile.print(","); // Because this is the end of the file, don't add a comma if it's unneccessary.
    }

    dataFile.println();  // End of line
    dataFile.close(); // stop writing.
    Serial.println("Data written to SD.");
  } else {
    Serial.println("Error opening telemetry.csv");
  } // might be better to put the test in setup() instead of checking every time... this is good in case of failure/writing too much data.
}


bool cast(int i) {
  // check the state of a given pin and return a boolean.
  // might be unneccessary -- originally was larger but I cut it down
  return digitalRead(i) == HIGH;
}