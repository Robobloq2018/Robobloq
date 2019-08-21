#include "RB_DIGITALDISPLAY.h"

uint8_t DIGITAL_tab[]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,0X00};


RB_DIGITALDISPLAY::RB_DIGITALDISPLAY(void):RB_SoftI2CMaster(0)
{	

   RB_SoftI2CMaster::SetMode(0);

}

RB_DIGITALDISPLAY::RB_DIGITALDISPLAY(uint8_t port):RB_SoftI2CMaster(port)
{	
  
   RB_SoftI2CMaster::SetMode(0);
 
}

void RB_DIGITALDISPLAY::RB_DIGITALDISPLAY_Init(void)
{
   RB_SoftI2CMaster::I2C_Star();
   RB_SoftI2CMaster::I2C_Write(0X48);
   RB_SoftI2CMaster::I2C_GetAck();
   RB_SoftI2CMaster::I2C_Write(0X31);
   RB_SoftI2CMaster::I2C_Stop();
  
  
}

void RB_DIGITALDISPLAY::RB_DIGITALDISPLAY_SET(uint8_t Address,uint8_t Data)
{  
   if(Address<1) return;
   RB_SoftI2CMaster::I2C_Star();
   RB_SoftI2CMaster::I2C_Write(0X68+2*(Address-1));
   RB_SoftI2CMaster::I2C_GetAck();
   if(Data&0X80)
     RB_SoftI2CMaster::I2C_Write(DIGITAL_tab[Data&0x7F]|0X80);
   else 
	 RB_SoftI2CMaster::I2C_Write(DIGITAL_tab[Data&0x7F]); 
   RB_SoftI2CMaster::I2C_Stop();
}

/*
   Clear All Digital Display
*/
void RB_DIGITALDISPLAY::Clear(void)
{
	RB_DIGITALDISPLAY_SET(0X68+2*0,0X10);
	RB_DIGITALDISPLAY_SET(0X68+2*1,0X10);
	RB_DIGITALDISPLAY_SET(0X68+2*2,0X10);
	RB_DIGITALDISPLAY_SET(0X68+2*3,0X10);
	
}