#include "RB_TEMPERATURE.h"


RB_TEMPERATURE :: RB_TEMPERATURE(void)
{
	
	
}

RB_TEMPERATURE:: RB_TEMPERATURE(uint8_t port):RB_Port(port)
{
    _DataPinS1 = RBPort[port].clk;
    _DataPinS2 = RBPort[port].dat;
}


void RB_TEMPERATURE::SetPin(uint8_t port)
{
	_DataPin = port;
	_ts.reset(port);
	
}

float 	RB_TEMPERATURE ::GET_TTEMPERATURE(uint8_t signal ,uint8_t robobloq_flag)
{
  
  
  byte  i;
  byte  present = 0;
  byte  type_s;
  byte  data[12];
  byte	addr[8];
  float celsius;
  long  time;
  
  
  if(robobloq_flag==1)
  {
     if(signal==1){
	      SetPin(_DataPinS1);
     }
     else if(signal==2){ 
	      SetPin(_DataPinS2);
	 }
  }
  else 
	      SetPin(signal); 

  if(_ts.reset()==0)return 0;
  _ts.skip();
  _ts.write(0x44) ;    
  
  time = millis();
  while(!_ts.readIO() )
  {
    if((millis()-time)>400){
		return 0;
	}
  }
  present = _ts.reset();
  _ts.skip();
  _ts.write(0xBE);
  for(i = 0; i < 5; i++)      // we need 9 bytes
  {
    data[i] = _ts.read();
  }

  int16_t rawTemperature = (data[1] << 8) | data[0];
  
  return( (float)rawTemperature * 0.0625); // 12 bit
		
}


