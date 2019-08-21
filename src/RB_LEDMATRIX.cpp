#include "RB_LEDMATRIX.h"

RB_LEDMatrix::RB_LEDMatrix(void) :RB_SoftI2CMaster(0)
{
  RB_SoftI2CMaster::SetMode(1);
}
RB_LEDMatrix::RB_LEDMatrix(uint8_t port):RB_SoftI2CMaster(port)
{ 
  RB_SoftI2CMaster::SetMode(1);
  I2C_Star();
  send(TM1680ID);   //  0xE7
  send(SYSDIS);  
  send(COM16NMOS);   //根据需求进行选择  COM8NMOS
  send(RCMODE1); 
  send(SYSEN);  
  send(LEDON); 
  I2C_Stop(); 
  
}

/***
    函数功能：页操作
***/
void RB_LEDMatrix:: RB_LEDMatrix_PageWrite(unsigned char faddr, unsigned char pdate[20]) 
{
    unsigned char  i=0,j=19,k=0;
    unsigned short ussdata =0 ;
    unsigned char  outdata[20];
    unsigned char  data;

    for(i=0; i<10; i++)
   {    
        ussdata = (unsigned short)pdate[2*i]<<8|pdate[2*i+1];
        ussdata = ((ussdata & 0xAAAA) >> 1) | ((ussdata & 0x5555) << 1);  
        ussdata = ((ussdata & 0xCCCC) >> 2) | ((ussdata & 0x3333) << 2);  
        ussdata = ((ussdata & 0xF0F0) >> 4) | ((ussdata & 0x0F0F) << 4);  
        ussdata = ((ussdata & 0xFF00) >> 8) | ((ussdata & 0x00FF) << 8);  
        
        ussdata = ussdata<<2;
        outdata[j--] = (unsigned char)(ussdata&0xff);
        outdata[j--]  = (unsigned char )((ussdata&0xff00)>>8);  
        
    }

    I2C_Star();
    
    send(TM1680ID);  //写TM1680器件地址
    send(faddr);  //eeprom 地址
   
    for(k=0; k<20; k++)
    {    
      data  = outdata[k];
      data=((data&0xCC)>>2) | ((data&0x33)<<2); 
      data=((data&0xAA)>>1) | ((data&0x55)<<1);   
      send(data);  //写       
    }
    
    I2C_Stop();
}
void RB_LEDMatrix::RB_LEDMatrix_PageAllWrite(unsigned char faddr,unsigned char sdate,unsigned char cnt)
{
    unsigned char i=0;
    unsigned char date=0;
    I2C_Star();
    
    send(TM1680ID);  
    send(faddr);  //eeprom 地址
    for(i=0; i<cnt; i++)
    {       
    send(sdate);  //写       
    }
    I2C_Stop();
} 
