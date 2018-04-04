#include "RB_LIGHTSENSOR.h"



RB_LightSensor::RB_LightSensor(void)
{
	
	
	
}

RB_LightSensor::RB_LightSensor(uint8_t port):RB_Port(port)
{
	
	_SigPin = RBPort[port].dat;
}

void RB_LightSensor::SetPin(uint8_t sigpin)
{
    _SigPin = sigpin;
}

uint16_t RB_LightSensor::GetLight(void)
{
	
	 pinMode(_SigPin,INPUT);
	 uint16_t value = 1024-analogRead(_SigPin);
	 return value;
}
