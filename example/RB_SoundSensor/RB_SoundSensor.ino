
#include "RB_SOUNDSENSOR.h"

RB_SoundSensor sound(2);

uint16_t  Sound_Value =0;


void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
   Sound_Value = sound.GetSound();
   Serial.println(Sound_Value);
   delay(500);
}
