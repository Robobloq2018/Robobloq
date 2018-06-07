#include "RB_Servo.h"


RB_Servo  RB_Servo(1);

void setup()
{
  RB_Servo.RB_SERVO_Write(150,150);   //控制两个舵机
  delay(500);
   RB_Servo.RB_SERVO_Write1(1);       //控制1号舵机
  delay(500);
   RB_Servo.RB_SERVO_Write2(1);       //控制2号舵机
  delay(500);
  
}

void loop()
{


}
