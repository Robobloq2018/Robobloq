#include "RB_PIRSENSOR.h"
uint16_t PIR_Value = 0;
RB_PirSensor  PIR(2);
void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
PIR_Value =PIR.GetPirSensor();
Serial.println(PIR_Value);
delay(1000);
}
