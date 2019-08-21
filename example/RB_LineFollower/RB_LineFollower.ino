

#include "RB_LINEFOLLOWER.h"

RB_LineFollower LineFollower(2);
uint8_t val;
void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
   val = LineFollower.ReadSensors();
   Serial.println(val);
   delay(600);
   
}
