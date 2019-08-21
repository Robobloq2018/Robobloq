#include "RB_MP3.h"


RB_MP3  MP3(3);

void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
 delay(1000);
  MP3.RB_MP3_Star(0x04,0x02);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
}
