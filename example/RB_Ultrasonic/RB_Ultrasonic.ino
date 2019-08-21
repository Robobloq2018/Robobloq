#include "RB_ULTRASONIC.h"
RB_Ultrasonic  ul(2);
double ul_distance =0;

void setup() {
  // put your setup code here, to run once:
    ul.RB_Ultrasonic_SetRGB(0x40,0xA1,200,0,0);
    delay(1000);
    ul.RB_Ultrasonic_SetRGB(0x40,0xA1,0,200,0);
    delay(1000);
     ul.RB_Ultrasonic_SetRGB(0x40,0xA1,0,0,200);
    delay(1000);  
    ul.RB_Ultrasonic_SetRGB(0x40,0xA1,0,0,0);
    Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
   ul_distance = ul.Uldistance();
   Serial.println(ul_distance);
   delay(500);
}
