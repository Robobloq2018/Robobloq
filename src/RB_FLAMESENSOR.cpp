#include "RB_FLAMESENSOR.h"


/*

*/

RB_FLAMESENSOR::RB_FLAMESENSOR(void)
{
	
	
	
}

RB_FLAMESENSOR::RB_FLAMESENSOR(uint8_t port):RB_Port(port)
{
	
	_SigPin_Digital = RBPort[port].clk;
	_SigPin_Analog  = RBPort[port].dat;
}

void RB_FLAMESENSOR::SetPin(uint8_t sigpin_dig,uint8_t sigpin_ana)
{
    _SigPin_Digital = sigpin_dig;
	_SigPin_Analog = sigpin_ana;
}


uint16_t RB_FLAMESENSOR::GetFlameAnalog(void)
{
	
	 pinMode(_SigPin_Analog,INPUT);
	 uint16_t value = analogRead(_SigPin_Analog);
	 return value;
}


uint8_t  RB_FLAMESENSOR:: GetFlameDValue(void)
{
	
	 pinMode(_SigPin_Digital,INPUT);
	 uint8_t value = digitalRead(_SigPin_Digital) ;
	 return _SigPin_Digital ;
	
}

uint16_t  RB_FLAMESENSOR::  GetFlameAValue(void)

{
    int i = 0;
    uint16_t Value[10];
    uint16_t max = 0,min =0;
    uint32_t sum = 0;
    uint16_t return_value = 0;
    for(i=0;i<10;i++) 
       {
          Value[i] = GetFlameAnalog();
          if(Value[i]>max)
             max = Value[i];
          if(Value[i]<min)
             min = Value[i];
          sum += Value[i];
       }
     sum = sum-max-min;
     return_value = sum/8;
     return return_value;
}
