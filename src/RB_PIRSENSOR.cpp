#include "RB_PIRSENSOR.h"


RB_PirSensor:: RB_PirSensor(void)
{
	
	
}

RB_PirSensor::RB_PirSensor(uint8_t port):RB_Port(port)
{
	
	_SigPin = RBPort[port].dat ;
	_ModePin = RBPort[port].clk ;
	pinMode(_SigPin,INPUT);
	pinMode(_ModePin,OUTPUT);
	digitalWrite(_ModePin,LOW);
}


void RB_PirSensor::SetPin(uint8_t sigpin,uint8_t modepin)
{
    _SigPin = sigpin ;
	_ModePin = modepin ;
    pinMode(_SigPin,INPUT);
	pinMode(_ModePin,OUTPUT);
	digitalWrite(_ModePin,LOW);
}
void RB_PirSensor::SetPirMode(uint8_t Mode)
{
	
	digitalWrite(_ModePin,Mode);
}

uint8_t RB_PirSensor::GetPirSensor(void)
{
	
	if((digitalRead(_SigPin)==0))
	{
		return 1;
	}
	else 
    {
		return 0;
	}
	
}
