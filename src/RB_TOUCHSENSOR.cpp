#include "RB_TOUCHSENSOR.h"


 



RB_TOUCHSENSOR :: RB_TOUCHSENSOR(void) :RB_SoftI2CMaster(0)
{
	RB_SoftI2CMaster::SetMode(1); 
	touchvalue = 0;
	
}

RB_TOUCHSENSOR::RB_TOUCHSENSOR(uint8_t port ):RB_SoftI2CMaster(port)
{
	RB_SoftI2CMaster::SetMode(1); 
    touchvalue = 0;
}




void RB_TOUCHSENSOR::RB_TOUCHSENSOR_ReadArray(uint8_t Register_Address,uint8_t *data,uint8_t datalen)
{
  beginTransmission(0x71);
  send(Register_Address);
  endTransmission();
  delay(10);
  RB_SoftI2CMaster::I2C_Star();
  RB_SoftI2CMaster::I2C_Write(0X71<<1|0x01);
  RB_SoftI2CMaster::I2C_GetAck();
  datalen=datalen-1;
  while(datalen--)
	{
		*data =I2C_Read(); 
		I2C_PutAck(0);
		data++;
	}
  *data =I2C_Read(); 
  RB_SoftI2CMaster::I2C_PutAck(1);
  RB_SoftI2CMaster::I2C_Stop();
}  


uint16_t  RB_TOUCHSENSOR::RB_TOUCHSENSOR_ReadValue(void)
{
  uint8_t touchsensorarray[4];  
  RB_TOUCHSENSOR_ReadArray(0xB0,touchsensorarray,4);
  if(touchsensorarray[0]==0xAA&&touchsensorarray[3]==0XBB)
   {
      touchvalue = ((uint16_t)touchsensorarray[1]*256)+touchsensorarray[2];
   }
  else
	  touchvalue =1;
  return touchvalue;
}


