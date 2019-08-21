#include "RB_LINEFOLLOWARRAY.h"


 



RB_LINEFOLLOWARRAY :: RB_LINEFOLLOWARRAY(void) :RB_SoftI2CMaster(0)
{
	RB_SoftI2CMaster::SetMode(1); 
	Last_Arry = 0;
	
}

RB_LINEFOLLOWARRAY::RB_LINEFOLLOWARRAY(uint8_t port ):RB_SoftI2CMaster(port)
{
	RB_SoftI2CMaster::SetMode(1); 
  Last_Arry = 0;
}




void RB_LINEFOLLOWARRAY::RB_LINEFOLLOWARRAY_ReadArray(uint8_t Register_Address,uint8_t *data,uint8_t datalen)
{
  beginTransmission(0x70);
  send(Register_Address);
  endTransmission();
  RB_SoftI2CMaster::I2C_Star();
  RB_SoftI2CMaster::I2C_Write(0X70<<1|0x01);
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


uint8_t  RB_LINEFOLLOWARRAY::RB_LINEFOLLOWARRAY_ReadValue(void)
{
  uint8_t linefollowarray[3];  
  RB_LINEFOLLOWARRAY_ReadArray(0xB0,linefollowarray,3);
  if(linefollowarray[0]==0xAA&&linefollowarray[2]==0XBB)
   {
      Last_Arry = linefollowarray[1];
   }
  else 
     return 0;
 
  return Last_Arry;
}


