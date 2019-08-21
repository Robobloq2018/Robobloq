#include "RB_LIGHTSENSOR.h"


RB_LightSensor LightSensor(2);
uint16_t LightValue = 0;


void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
   
}

void loop() {
  // put your main code here, to run repeatedly:
   LightValue =  LightSensor.GetLight();
   Serial.println(LightValue);
   delay(500);
}
