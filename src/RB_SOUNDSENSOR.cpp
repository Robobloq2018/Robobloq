#include "RB_SOUNDSENSOR.h"



RB_SoundSensor::RB_SoundSensor(void)
{
	
	
	
}

RB_SoundSensor::RB_SoundSensor(uint8_t port):RB_Port(port)
{
	
	_SigPin = RBPort[port].dat;
}

void RB_SoundSensor::SetPin(uint8_t sigpin)
{
    _SigPin = sigpin;
}

uint16_t RB_SoundSensor::GetSound(void)
{
	
	 pinMode(_SigPin,INPUT);
	 uint16_t value = analogRead(_SigPin);
	 return value;
}
