#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "RB_SOFTI2CMASTER.h"

#include <util/delay.h>
#include <string.h>

#define  I2C_ACK  1 
#define  I2C_NAK  0
#define  i2cbitdelay 1 


RB_SoftI2CMaster :: RB_SoftI2CMaster(uint8_t _sdapin,uint8_t _sclpin):RB_Port(0)
 {
  _SDA = _sdapin;
  _SCL  = _sclpin; 
  
}
  RB_SoftI2CMaster :: RB_SoftI2CMaster(uint8_t port):RB_Port(port)
 {
  _SDA = RBPort[port].dat;
  _SCL  = RBPort[port].clk; 
  
}

void RB_SoftI2CMaster::I2C_Star(void)
{
   pinMode(_SCL,OUTPUT);
   pinMode(_SDA,OUTPUT);
   digitalWrite(_SDA, HIGH); _delay_us(15);
   digitalWrite(_SCL, HIGH); _delay_us(15);
   digitalWrite(_SDA, LOW); _delay_us(15);
   digitalWrite(_SCL, LOW);_delay_us(15);
}

uint8_t RB_SoftI2CMaster::I2C_Write(uint8_t dat)
{
   pinMode(_SCL,OUTPUT);
   pinMode(_SDA,OUTPUT);
   for(uint8_t i =0;i<8;i++)
   {
      if(dat&0x80)
         {
          digitalWrite(_SDA, HIGH);}
      else {
         digitalWrite(_SDA, LOW);}
      _delay_us(15);
      dat<<=1;
      digitalWrite(_SCL, HIGH);_delay_us(15);
      digitalWrite(_SCL, LOW);_delay_us(15);
    } 
}
uint8_t RB_SoftI2CMaster::I2C_Read(void)
{ 
  uint8_t dat;
  pinMode(_SDA,INPUT);
  for(uint8_t i=0;i<8;i++)
    {
      digitalWrite(_SCL, HIGH);
      dat<<=1;
      if(digitalRead(_SDA)){
         dat|=0x01;
      }
      _delay_us(15);
      digitalWrite(_SCL, LOW);;_delay_us(15);
    }
   return   dat;
  }

  uint8_t RB_SoftI2CMaster::I2C_GetAck(void)
{
   uint8_t ack=0;
   uint8_t cnt =0;
   pinMode(_SDA,INPUT);
     //总线准备，接受应答
   digitalWrite(_SDA, HIGH);_delay_us(15);
   digitalWrite(_SCL, HIGH);_delay_us(15);
   if(digitalRead(_SDA)!=0){
        ack = 1; 
   }
   digitalWrite(_SCL, LOW);_delay_us(15);
   return ack;
}
void RB_SoftI2CMaster::I2C_PutAck(uint8_t ack)
{
   pinMode(_SDA,OUTPUT);
   if(ack==0)
    {
     digitalWrite(_SDA, LOW);  //
    }
    else
    {
     digitalWrite(_SDA, HIGH);
    }
    digitalWrite(_SCL, HIGH);_delay_us(15);
    digitalWrite(_SCL, LOW);_delay_us(15);
    digitalWrite(_SDA, HIGH);
}
void RB_SoftI2CMaster::I2C_Stop(void )
{
   pinMode(_SDA,OUTPUT);
  digitalWrite(_SCL, LOW);_delay_us(15);
  digitalWrite(_SDA, LOW);_delay_us(15);
  digitalWrite(_SCL, HIGH);_delay_us(15);
  digitalWrite(_SDA, HIGH);_delay_us(20);
}
void RB_SoftI2CMaster::beginTransmission(uint8_t slaveaddress)
{
  I2C_Star();
  I2C_Write(slaveaddress<<1|0x00);
  I2C_GetAck();
}

void RB_SoftI2CMaster::endTransmission(void)
{
  I2C_Stop();
 }
 uint8_t RB_SoftI2CMaster::send(uint8_t data)
{ 

  I2C_Write(data);
  I2C_GetAck();
 }
 
  
   uint8_t RB_SoftI2CMaster::Write_OneByte(uint8_t slaveaddress,uint8_t regaddress,uint8_t data)
 {  

     
    I2C_Star();
    I2C_Write(slaveaddress<<1|0x00);
    I2C_GetAck();
    I2C_Write(regaddress|0x80);
    I2C_GetAck();
  I2C_Write(data);
  I2C_GetAck();
    I2C_Stop();

    return  data;
  
  }
  
  uint8_t RB_SoftI2CMaster::Read_OneByte(uint8_t slaveaddress,uint8_t regaddress)
 {  

     uint8_t  data= 0;
    I2C_Star();
    I2C_Write(slaveaddress<<1|0x00);
   
    I2C_GetAck();
    I2C_Write(regaddress|0x80);

    I2C_GetAck();
    I2C_Star();
    I2C_Write(slaveaddress<<1|0x01);

    I2C_GetAck();
    data = I2C_Read();
    I2C_PutAck(1);

    I2C_Stop();

    return  data;
  
  }

  uint16_t RB_SoftI2CMaster::Read_TwoByte(uint8_t slaveaddress,uint8_t regaddress)
 {  
    uint16_t  x,t;
    uint16_t  data= 0;
    I2C_Star();
    I2C_Write(slaveaddress<<1|0x00);
  
    I2C_GetAck();
    I2C_Write(regaddress|0x80);

    I2C_GetAck();
    I2C_Star();
    I2C_Write(slaveaddress<<1|0x01);

    I2C_GetAck();
    t = I2C_Read();
    I2C_PutAck(0);
     x = I2C_Read();
    I2C_PutAck(1);
    I2C_Stop();
    data = x<<8|t;
    
    return  data;
  
  }
