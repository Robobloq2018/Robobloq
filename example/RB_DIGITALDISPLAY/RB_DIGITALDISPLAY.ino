#include "RB_DIGITALDISPLAY.h"

RB_DIGITALDISPLAY  DISPLAY1(1);
void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
   delay(50);
   DISPLAY1.RB_DIGITALDISPLAY_Init();
   DISPLAY1.RB_DIGITALDISPLAY_SET(3,5);
   delay(500);
   DISPLAY1.RB_DIGITALDISPLAY_SET(2,2|0x80);
   delay(500);
   DISPLAY1.RB_DIGITALDISPLAY_SET(1,1);
   delay(500);
   DISPLAY1.RB_DIGITALDISPLAY_SET(4,0);
}

void loop() {
  // put your main code here, to run repeatedly:

}
