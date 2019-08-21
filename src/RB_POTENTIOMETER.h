#ifndef RB_POTENTIOMETER_H
#define RB_POTENTIOMETER_H


#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "RB_PORT.h" 


class RB_POTENTIOMETER:public RB_Port
{
	public:
	              RB_POTENTIOMETER(void);
	              RB_POTENTIOMETER(uint8_t port);
         void     SetPin(uint8_t sigpin);
	     uint16_t GetPotentiometer(void); 

 private:
         uint8_t  _SigPin;
};


#endif
