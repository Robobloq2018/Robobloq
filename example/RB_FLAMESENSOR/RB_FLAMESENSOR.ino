#include "RB_FLAMESENSOR.h"


RB_FLAMESENSOR    Flame(2);


uint16_t flameavalue = 0;
uint16_t flamedvalue = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
   flameavalue = Flame.GetFlameAValue();
   flamedvalue = Flame.GetFlameDValue();
   Serial.print("Robobloq FlameSensor analog value:");
   Serial.println(flameavalue);
   Serial.print("Robobloq FlameSensor digital value:");
   Serial.println(flamedvalue);
   delay(500);
}
