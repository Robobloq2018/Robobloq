#include "RB_DCMOTOR.h"


RB_DCMotor::RB_DCMotor(void):RB_SoftI2CMaster(0)
{	
   pinMode(20,OUTPUT);
   digitalWrite(MOTOR_Charge_Pin,HIGH);  
}

RB_DCMotor::RB_DCMotor(uint8_t port):RB_SoftI2CMaster(port)
{	
   
   pinMode(20,OUTPUT);
   digitalWrite(MOTOR_Charge_Pin,HIGH);  
}

void RB_DCMotor::RB_DCMotor_SetSpeed(uint8_t SlaveAddress,uint8_t RegisterAddress,uint8_t Index,uint8_t Dir,uint8_t Speed)
{
   beginTransmission(SlaveAddress);
   send(RegisterAddress);
   send(Index);
   send(Dir);
   send(Speed);
   endTransmission();
}
