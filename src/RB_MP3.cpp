#include "RB_MP3.h"

unsigned char Senddata[10];


RB_MP3::RB_MP3(void):RB_Port(),SoftwareSerial(NC, NC)
{
	
	
	
}


RB_MP3::RB_MP3(uint8_t port):RB_Port(port),SoftwareSerial(RBPort[port].clk,RBPort[port].dat)
{
	_mode = 3;
	SoftwareSerial::begin(38400);
  
	
}
uint8_t  RB_MP3::RB_MP3_Mode(void)
{
   return _mode;   
  
}
void    RB_MP3::RB_MP3_Set(uint8_t mode)
{
  unsigned char i =0 ,check = 0;
  _mode = mode ;
  Senddata[0] = 0xAA;
  Senddata[1] = 0x02;
  Senddata[2] = 0x01;
  Senddata[3] = mode;
  for(i=0;i<4;i++)
  {
    check = (check^Senddata[i]);
  }
  Senddata[4] =~check;
  for(i=0;i<5;i++)
  {
   SoftwareSerial::write(Senddata[i]);
    }
  _mode = mode;
  delay(50);
  
}
void RB_MP3::RB_MP3_Star(uint8_t mode ,uint8_t data)
{   

     unsigned char i =0 ,check = 0;
	Senddata[0] = 0xAA;
	Senddata[1] = mode;
	Senddata[2] = 0x01;
	Senddata[3] = data;
	for(i=0;i<4;i++)
	{
		check = (check^Senddata[i]);
	}
	Senddata[4] =~check;
	for(i=0;i<5;i++)
	{
	 SoftwareSerial::write(Senddata[i]);
    }
	delay(50);
}

void RB_MP3::RB_MP3_END(void)
{
	
	SoftwareSerial::end();
	
}
