#include <FreeSixIMU.h>
//#include <FIMU_ADXL345.h>
//#include <FIMU_ITG3200.h>
#include <aADXL345.h>
#include <aITG3200.h>
#include <Wire.h>
#include <SensorDataFilter.h>

//#include "CommunicationUtils.h"

float angles[3]; // yaw pitch roll
float q[4]; //hold q values

int brushing = 0;
int brushingAcc = 0;
int threshold = 100;

SensorDataFilter* d0 = new SensorDataFilter(5);
SensorDataFilter* d1 = new SensorDataFilter(5);
SensorDataFilter* d2 = new SensorDataFilter(5);

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

//#define DEBUG
#define BLUETOOTH

void setup() { 
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  
  #ifdef BLUETOOTH
  Serial3.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  #endif

  Wire.begin();
  
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}

void loop() { 
/*
  sixDOF.getEuler(angles);
  Serial.print(angles[0]);
  Serial.print("|");  
  Serial.print(angles[1]);
  Serial.print("|");
  Serial.print(angles[2]);
  Serial.print("|");
  */

  /*
  int values[6];
  sixDOF.getRawValues(&values[0]);
  Serial.print(values[0]);
  Serial.print("|");
  Serial.print(values[1]);
  Serial.print("|");
  Serial.print(values[2]);
  Serial.print("|");
  Serial.print(values[3]);
  Serial.print("|");
  Serial.print(values[4]);
  Serial.print("|");
  Serial.print(values[5]);
  Serial.print("|");
  */
  
  sixDOF.getQ(q);

  #ifdef BLUETOOTH  
  Serial3.print(q[0], 6);
  Serial3.print("|");
  Serial3.print(q[1], 6);
  Serial3.print("|");
  Serial3.print(q[2], 6);
  Serial3.print("|");
  Serial3.print(q[3], 6);
  Serial3.print("|");

  checkBrushing();
  Serial3.print(brushing);
  Serial3.print("|");
  Serial3.print(brushingAcc);
  Serial3.print("|");

  Serial3.println(""); //line break
  #else
  Serial.print(q[0], 6);
  Serial.print("|");
  Serial.print(q[1], 6);
  Serial.print("|");
  Serial.print(q[2], 6);
  Serial.print("|");
  Serial.print(q[3], 6);
  Serial.print("|");

  checkBrushing();
  Serial.print(brushing);
  Serial.print("|");
  Serial.print(brushingAcc);
  Serial.print("|");

  Serial.println(""); //line break
  #endif

  delay(60); 
}

void checkBrushing() {
  int accval[3];
  sixDOF.acc.readAccel(&accval[0], &accval[1], &accval[2]);

  d0->add(accval[0]);
  d1->add(accval[1]);
  d2->add(accval[2]);

  int v0 = d0->getVariation();
  int v1 = d1->getVariation();
  int v2 = d2->getVariation();

  brushing = threshold;

  if (v0 > brushing) {
    brushing = v0;
  }
  if (v2 > brushing) {
    brushing = v2;
  }
  if (v1 > brushing) {
    // Brush stroke is most perpendicular to the brush, compared to other directions and threshold.
    brushing = v1;
  } else {
    brushing = 0;
  }
  brushingAcc = accval[1];
}

