#ifndef RB_SERVO_H
#define RB_SERVO_H

 
 
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "RB_PORT.h" 
#include "Servo.h"



class RB_SERVO:public RB_Port
{
	
public:	
    RB_SERVO(void);
    RB_SERVO(uint8_t Servo1_pin,uint8_t Servo2_pin);
    RB_SERVO(uint8_t port);
    void RB_SERVO::RB_SERVO_INIT(void);
    uint8_t  RB_SERVO_Attach1(int pin, int min, int max);
    uint8_t  RB_SERVO_Attach2(int pin, int min, int max);
    void RB_SERVO_Detach1(void);
    void RB_SERVO_Detach2(void);
    void RB_SERVO_Detach(void);
	void RB_SERVO_Write_Servo1(uint8_t value);
	void RB_SERVO_Write_Servo2(uint8_t value);
	void RB_SERVO_Write(uint8_t value1,uint8_t value2);
		
private :
   uint8_t _Servo1_pin;
   uint8_t _Servo2_pin;
};
 


#endif

