#include "RB_DCMOTOR.h"


RB_DCMotor::RB_DCMotor(void):RB_SoftI2CMaster(0)
{	
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
   pinMode(20,OUTPUT);
   digitalWrite(MOTOR_Charge_Pin,HIGH);  
#endif
   RB_SoftI2CMaster::SetMode(0);
}

RB_DCMotor::RB_DCMotor(uint8_t port):RB_SoftI2CMaster(port)
{	
   
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
   pinMode(20,OUTPUT);
   digitalWrite(MOTOR_Charge_Pin,HIGH);  
#endif
   RB_SoftI2CMaster::SetMode(0);
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
