#ifndef _RB_TEMPERATURE_H
#define _RB_TEMPERATURE_H


#include<stdint.h>
#include<stdbool.h>
#include<Arduino.h>

#include "RB_PORT.h" 
#include "RB_OneWire.h"


class RB_TEMPERATURE:public RB_Port
{
	public :
	        RB_TEMPERATURE(void);
 	        RB_TEMPERATURE(uint8_t port);
	        void  SetPin(uint8_t port);
			float GET_TTEMPERATURE(uint8_t signal ,uint8_t robobloq_flag=1);
	private :
            uint8_t _DataPin;
            uint8_t _DataPinS1;
            uint8_t _DataPinS2;	
	        RB_OneWire _ts;
	
    
};






#endif
