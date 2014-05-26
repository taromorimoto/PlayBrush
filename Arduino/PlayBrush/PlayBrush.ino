#include <SPI.h>
#include <Wire.h>
#include <SensorDataFilter.h>
#include <DOFHelper.h>

//#include "CommunicationUtils.h"

float angles[3]; // yaw pitch roll
float q[4]; //hold q values

int brushing = 0;
int brushingAcc = 0;
int threshold = 100;

SensorDataFilter* d0 = new SensorDataFilter(5);
SensorDataFilter* d1 = new SensorDataFilter(5);
SensorDataFilter* d2 = new SensorDataFilter(5);

DOFHelper* dof;

//#define DEBUG
#define BLUETOOTH

void setup() { 
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  
  #ifdef BLUETOOTH
  Serial3.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  #endif
  
  //Wire.begin();
  
  dof = new DOFHelper();
}

void loop() { 
  dof->update();

  #ifdef BLUETOOTH  
  
  Serial3.print(dof->q[0]);
  Serial3.print("|");
  Serial3.print(dof->q[1]);
  Serial3.print("|");
  Serial3.print(dof->q[2]);
  Serial3.print("|");
  Serial3.print(dof->q[3]);
  Serial3.print("|");

  checkBrushing();
  Serial3.print(brushing);
  Serial3.print("|");
  Serial3.print(brushingAcc);
  Serial3.print("|");

  Serial3.print(dof->yaw);
  Serial3.print("|");
  Serial3.print(dof->pitch);
  Serial3.print("|");
  Serial3.print(dof->roll);
  Serial3.print("|");  
  
  Serial3.println(""); //line break
  
  #else
  
  Serial.print(dof->q[0]);
  Serial.print("|");
  Serial.print(dof->q[1]);
  Serial.print("|");
  Serial.print(dof->q[2]);
  Serial.print("|");
  Serial.print(dof->q[3]);
  Serial.print("|");

  checkBrushing();
  Serial.print(brushing);
  Serial.print("|");
  Serial.print(brushingAcc);
  Serial.print("|");

  Serial.print(dof->yaw);
  Serial.print("|");
  Serial.print(dof->pitch);
  Serial.print("|");
  Serial.print(dof->roll);
  Serial.print("|");  
  
  Serial.println(""); //line break
  
  #endif
  
  #ifdef DEBUG  
  dof->printValues();
  #endif

  delay(60); 
}

void checkBrushing() {
  d0->add(dof->ax*100);
  d1->add(dof->ay*100);
  d2->add(dof->az*100);
  

  int v0 = d0->getVariation();
  int v1 = d1->getVariation();
  int v2 = d2->getVariation();

/*
  Serial.print(v0);
  Serial.print(" ");
  Serial.print(v1);
  Serial.print(" ");
  Serial.print(v2);
  Serial.println(" ");
*/
  brushing = threshold;

  if (v1 > brushing) {
    brushing = v1;
  }
  if (v2 > brushing) {
    brushing = v2;
  }
  if (v0 > brushing) {
    // Brush stroke is most perpendicular to the brush, compared to other directions and threshold.
    brushing = v0;
  } else {
    brushing = 0;
  }
  brushingAcc = dof->ax*500;
}
