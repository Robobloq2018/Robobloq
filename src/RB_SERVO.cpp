/*
*
*
*
*
*
*/

#include "RB_SERVO.h"





#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
Servo RB_Servo1;
Servo RB_Servo2;
#endif


RB_SERVO::RB_SERVO(uint8_t Servo1_pin,uint8_t Servo2_pin)
{
	
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
   pinMode(20,OUTPUT);
   digitalWrite(MOTOR_Charge_Pin,HIGH);  
   _Servo1_pin  = Servo1_pin;
   _Servo2_pin  = Servo2_pin;
    pinMode(_Servo1_pin,OUTPUT);
    pinMode(_Servo2_pin,OUTPUT);
#else 
	_Servo1_pin  = Servo1_pin;
	_Servo2_pin  = Servo2_pin;
	pinMode(_Servo1_pin,OUTPUT);
    pinMode(_Servo2_pin,OUTPUT);
#endif
     
	
}

RB_SERVO::RB_SERVO(uint8_t port): RB_Port(port)
{  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)     
   pinMode(20,OUTPUT);
   digitalWrite(MOTOR_Charge_Pin,HIGH);  
   _Servo1_pin = RBPort[port].clk;
   _Servo2_pin = RBPort[port].dat;
   pinMode(_Servo1_pin,OUTPUT);
   pinMode(_Servo2_pin,OUTPUT);
#else
    _Servo1_pin = RBPort[port].clk;
	_Servo2_pin = RBPort[port].dat;
	pinMode(_Servo1_pin,OUTPUT);
    pinMode(_Servo2_pin,OUTPUT);
#endif	
}

void RB_SERVO::RB_SERVO_INIT(void)
{   

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)  	
	RB_Servo1.attach(_Servo1_pin);
	RB_Servo2.attach(_Servo2_pin);
#else
     TCCR2A = _BV(COM2A1) |_BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
     TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);	
#endif	
}

uint8_t  RB_SERVO::RB_SERVO_Attach1(int pin, int min, int max)
{
	#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)  
	RB_Servo1.attach(_Servo1_pin,min,max);
	#endif
}
uint8_t  RB_SERVO::RB_SERVO_Attach2(int pin, int min, int max)
{    
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)  
	RB_Servo2.attach(_Servo1_pin,min,max);
	#endif
}
void RB_SERVO::RB_SERVO_Detach1(void)
{  
   #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)    
   RB_Servo1.detach();
   #endif
  }
void RB_SERVO::RB_SERVO_Detach2(void)
{  
   #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)    
   RB_Servo2.detach();
   #endif
}
void RB_SERVO::RB_SERVO_Detach(void)
{
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)    
  RB_SERVO::RB_SERVO_Detach1();
  RB_SERVO::RB_SERVO_Detach2();
  #endif
}  

void RB_SERVO::RB_SERVO_Write_Servo1(uint8_t value)
{   
    float  data = float(value)/1.27;
   value = uint8_t(data);
    if(value>180)
         value = 180;
    else if(value<0)
         value = 0;
	#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)    
    RB_Servo1.write(value);
	#else 
    if(value==0)
       OCR2B = 7 ;
    else if(value==180)
        OCR2B = 38 ;
    else OCR2B = 8+ value/6; 
    #endif	
}
void RB_SERVO::RB_SERVO_Write_Servo2(uint8_t value)
{   
    float  data = float(value)/1.27;
    value = uint8_t(data);
	
    if(value>180)
         value = 180;
    else if(value<0)
         value = 0;
	#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)  
    RB_Servo2.write(value);
	#else
    if(value==0)
       OCR2A = 7 ;
    else if(value==180)
        OCR2A = 38 ;
    else OCR2A = 8+ value/6; 
	#endif
	
}
void RB_SERVO::RB_SERVO_Write(uint8_t value1,uint8_t value2)
{   
 
   RB_SERVO_Write_Servo1(value1);
   RB_SERVO_Write_Servo2(value2);

}   



