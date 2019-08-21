#include "RB_ENCONDERMOTOR.h" 

#include "RB_ENCONDERMOTOR.h" 

RB_EncoderMotor  M1(1);
RB_EncoderMotor  M2(2);

void setup() {
  // put your setup code here, to run once:
    M1.SetMotionMode(0);
    M2.SetMotionMode(0);
    pinMode(20,OUTPUT);
    digitalWrite(20,HIGH); 
}  

void loop() {
  // put your main code here, to run repeatedly:
   M1.SetMotorPwm(30);
   M2.SetMotorPwm(33);
   delay(5000);

   
   M1.SetMotorPwm(-35);
   M2.SetMotorPwm(-35);
   delay(5000); 
 
  
}
