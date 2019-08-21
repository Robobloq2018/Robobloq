#include "RB_RGBLED.h"
RB_RGBLed RGBLED(2);
void setup() {
  // put your setup code here, to run once:
   RGBLED.setColor(1,50,0,0);
   RGBLED.show();
   delay(1000);
   RGBLED.setColor(2,0,50,0);
   RGBLED.show();
   delay(1000);
   RGBLED.setColor(3,0,0,50);
   RGBLED.show();
   delay(1000);
   RGBLED.setColor(4,0,0,0);
   RGBLED.show();
   delay(1000);
   
    RGBLED.setColor(1,50,0,0);
    RGBLED.show();
   delay(1000);
   RGBLED.setColor(2,0,50,0);
   RGBLED.show();
   delay(1000);
   RGBLED.setColor(3,0,0,50);
   RGBLED.show();
   delay(1000);
   RGBLED.setColor(4,0,0,0);
   RGBLED.show();
   delay(1000);
   
    RGBLED.setColor(0,50,0,0);
    RGBLED.show();
   delay(1000);
   RGBLED.setColor(0,0,50,0);
   RGBLED.show();
   delay(1000);
   RGBLED.setColor(0,0,0,50);
   RGBLED.show();
   delay(1000);
   RGBLED.setColor(0,0,0,0);
   RGBLED.show();
   delay(1000);
    RGBLED.setColor(0,255,255,255);
   RGBLED.show();
   delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
