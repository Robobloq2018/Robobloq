#ifndef RB_SoundSensor_H
#define RB_SoundSensor_H


#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "RB_PORT.h" 


class RB_SoundSensor:public RB_Port
{
	public:
	      RB_SoundSensor(void);
		    RB_SoundSensor(uint8_t port);
		    void SetPin(uint8_t sigpin);
	      uint16_t GetSound(void); 
	      uint16_t GetSoundValue(void); 
  private:
       uint8_t  _SigPin;
};


#endif
