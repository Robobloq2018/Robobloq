#ifndef RB_PIRSENSOR_H
#define RB_PIRSENSOR_H


#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "RB_PORT.h" 

class RB_PirSensor:public RB_Port
{
	public:
	   RB_PirSensor(void);
	   RB_PirSensor(uint8_t port);
	   void SetPin(uint8_t sigpin,uint8_t modepin);
	   void SetPirMode(uint8_t Mode);
	   uint8_t GetPirSensor(void);
	private :
     volatile uint8_t _SigPin;
     volatile uint8_t _ModePin;
};





#endif




