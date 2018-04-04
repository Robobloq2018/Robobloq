#include "RB_ENCONDERMOTOR.h" 

RB_EncoderMotor  M1(1);
RB_EncoderMotor  M2(2);

void setup() {
  // put your setup code here, to run once:
    M1.SetMotionMode(0);
    M2.SetMotionMode(0);
}  

void loop() {
  // put your main code here, to run repeatedly:
   M1.SetMotorPwm(90);
   M2.SetMotorPwm(90);
   delay(500); 
   M1.SetMotorPwm(70);
   M2.SetMotorPwm(70);
   delay(500); 
   M1.SetMotorPwm(50);
   M2.SetMotorPwm(50);
   delay(500); 
   M1.SetMotorPwm(0);
   M2.SetMotorPwm(0);
   delay(500); 
   M1.SetMotorPwm(-60);
   M2.SetMotorPwm(-60);
   delay(500); 
   M1.SetMotorPwm(-80);
   M2.SetMotorPwm(-80);
   delay(500); 
   M1.SetMotorPwm(-90);
   M2.SetMotorPwm(-90);
    delay(500); 
       M1.SetMotorPwm(90);
   M2.SetMotorPwm(-90);
    delay(500); 
     M1.SetMotorPwm(-90);
   M2.SetMotorPwm(90);
    delay(500);
       M1.SetMotorPwm(0);
   M2.SetMotorPwm(0); 
   delay(500);
}
