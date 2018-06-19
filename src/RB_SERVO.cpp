/*
*
*
*
*
*
*/

#include "RB_SERVO.h"

Servo RB_Servo1;
Servo RB_Servo2;

RB_SERVO::RB_SERVO(uint8_t Servo1_pin,uint8_t Servo2_pin)
{
	 pinMode(20,OUTPUT);
   digitalWrite(MOTOR_Charge_Pin,HIGH);  
	_Servo1_pin  = Servo1_pin;
	_Servo2_pin  = Servo2_pin;
	pinMode(_Servo1_pin,OUTPUT);
  pinMode(_Servo2_pin,OUTPUT);

	
}

RB_SERVO::RB_SERVO(uint8_t port): RB_Port(port)
{       
	 pinMode(20,OUTPUT);
   digitalWrite(MOTOR_Charge_Pin,HIGH);  
	_Servo1_pin = RBPort[port].clk;
	_Servo2_pin = RBPort[port].dat;
	pinMode(_Servo1_pin,OUTPUT);
  pinMode(_Servo2_pin,OUTPUT);
	
}

void RB_SERVO::RB_SERVO_INIT(void)
{   
	 RB_Servo1.attach(_Servo1_pin);
	RB_Servo2.attach(_Servo2_pin);
	RB_Servo1.write(90);
	RB_Servo1.write(90);
	
	
}

uint8_t  RB_SERVO::RB_SERVO_Attach1(int pin, int min, int max)
{
	RB_Servo1.attach(_Servo1_pin,min,max);
}
uint8_t  RB_SERVO::RB_SERVO_Attach2(int pin, int min, int max)
{
	RB_Servo2.attach(_Servo1_pin,min,max);
}
void RB_SERVO::RB_SERVO_Detach1(void)
{
   RB_Servo1.detach();
  }
void RB_SERVO::RB_SERVO_Detach2(void)
{
   RB_Servo2.detach();
}
void RB_SERVO::RB_SERVO_Detach(void)
{
  RB_SERVO::RB_SERVO_Detach1();
  RB_SERVO::RB_SERVO_Detach2();
}  

void RB_SERVO::RB_SERVO_Write_Servo1(uint8_t value)
{   if(value>180)
         value = 180;
    else if(value<0)
         value = 0;
    RB_Servo1.write(value);
}
void RB_SERVO::RB_SERVO_Write_Servo2(uint8_t value)
{   
    if(value>180)
         value = 180;
    else if(value<0)
         value = 0;
    RB_Servo2.write(value);
}
void RB_SERVO::RB_SERVO_Write(uint8_t value1,uint8_t value2)
{
   RB_Servo1.write(value1);
   RB_Servo2.write(value2);   
}
void RB_SERVO::RB_SERVO_WriteMicroseconds1(int value)
{
	RB_Servo1.writeMicroseconds(value);
}
void RB_SERVO::RB_SERVO_WriteMicroseconds2(int value)
{
	RB_Servo2.writeMicroseconds(value);
}

void RB_SERVO::RB_SERVO_WriteMicroseconds(int value1,int value2)
{
  RB_Servo1.writeMicroseconds(value1);
  RB_Servo2.writeMicroseconds(value2);
}

