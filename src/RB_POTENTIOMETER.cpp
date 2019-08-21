#include "RB_POTENTIOMETER.h"


/*

*/

RB_POTENTIOMETER::RB_POTENTIOMETER(void)
{
	
	
	
}

RB_POTENTIOMETER::RB_POTENTIOMETER(uint8_t port):RB_Port(port)
{
	
	_SigPin = RBPort[port].dat;
}

void RB_POTENTIOMETER::SetPin(uint8_t sigpin)
{
    _SigPin = sigpin;
}


uint16_t RB_POTENTIOMETER::GetPotentiometer(void)
{
	
	 pinMode(_SigPin,INPUT);
	 uint16_t value = analogRead(_SigPin);
	 return value;
}




