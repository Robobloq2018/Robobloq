#include "RB_SERVO.h"



RB_SERVO  Srvo(8);
void setup()
{
  Srvo.RB_SERVO_INIT();
  Srvo.RB_SERVO_Write(150,150);
  delay(300);
  
}

void loop()
{
   Srvo.RB_SERVO_Write(90,90);
   delay(1000);
   Srvo.RB_SERVO_Write(0,0);
   delay(1000);
   Srvo.RB_SERVO_Write(180,180);
  delay(1000);

}
