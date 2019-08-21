#ifndef RB_JOYSTICK_H
#define RB_JOYSTICK_H


#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <math.h>


#include "RB_PORT.h" 

#define CENTER_VALUE    (490)


class RB_JOYSTICK:public RB_Port
{
	public:
	             RB_JOYSTICK(void);
	             RB_JOYSTICK(uint8_t port);
           void  SetPin(uint8_t sigpin_x,uint8_t sigpin_y);
        int16_t  ReadJoystickX(void);
        int16_t  ReadJoystickY(void);
        int16_t  CalCenterValue(int16_t x_offset,int16_t y_offset);
		float    OffCenter(void);

 private:
        uint8_t  _SigPin_X;
	    uint8_t  _SigPin_Y;
		static volatile int16_t _X_offset ;
		static volatile int16_t _Y_offset ;
		
		
};


#endif
