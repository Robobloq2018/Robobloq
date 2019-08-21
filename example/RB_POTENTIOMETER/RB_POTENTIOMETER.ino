#include "RB_POTENTIOMETER.h"

RB_POTENTIOMETER  pot(2);

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
   
}

void loop() {
  // put your main code here, to run repeatedly:
   Serial.print("Robobloq Potentiometer Value:");
   Serial.println(pot.GetPotentiometer());
   delay(30);
}
