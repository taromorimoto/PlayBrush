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

void setup() { 
  Serial.begin(115200);
  //Serial.begin(38400);
  
  //Wire.begin();
  
  dof = new DOFHelper();
}

void loop() { 
  dof->update();
  
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
  
  //dof->printValues();

  delay(50); 
}

void checkBrushing() {
  d0->add(dof->ax*10);
  d1->add(dof->ay*10);
  d2->add(dof->az*10);

  int v0 = d0->getVariation();
  int v1 = d1->getVariation();
  int v2 = d2->getVariation();

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
  brushingAcc = dof->ax*10;
}
