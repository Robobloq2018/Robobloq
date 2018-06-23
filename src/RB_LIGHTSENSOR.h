#ifndef RB_LightSensor_H
#define RB_LightSensor_H


#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "RB_PORT.h" 


class RB_LightSensor:public RB_Port
{
	public:
	     RB_LightSensor(void);
	     RB_LightSensor(uint8_t port);
             void SetPin(uint8_t sigpin);
	     uint16_t GetLight(void); 
	     uint16_t GetLightValue(void);
 private:
       uint8_t  _SigPin;
};


#endif
