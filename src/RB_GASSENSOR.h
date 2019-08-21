#ifndef RB_GASSENSOR_H
#define RB_GASSENSOR_H


#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "RB_PORT.h" 


class RB_GASSENSOR:public RB_Port
{
	public:
	     RB_GASSENSOR(void);
	     RB_GASSENSOR(uint8_t port);
             void SetPin(uint8_t sigpin);
	     uint16_t GetGas(void); 
	     uint16_t GetGasValue(void);
 private:
       uint8_t  _SigPin;
};


#endif
