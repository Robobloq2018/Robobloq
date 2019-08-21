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
  _MODE = 0;
  
}
  RB_SoftI2CMaster :: RB_SoftI2CMaster(uint8_t port):RB_Port(port)
 {
  _SDA = RBPort[port].dat;
  _SCL  = RBPort[port].clk; 
  _MODE = 0;
}
void RB_SoftI2CMaster:: SetMode(uint8_t mode)
{
  _MODE  = mode;
 }
void RB_SoftI2CMaster::I2C_Star(void)
{
   pinMode(_SCL,OUTPUT);
   pinMode(_SDA,OUTPUT);
   if(_MODE==1)
  {
   _delay_us(150);
   digitalWrite(_SDA, HIGH); 
   digitalWrite(_SCL, HIGH);
   digitalWrite(_SDA, LOW); 
   digitalWrite(_SCL, LOW);
   }
   else 
  {
   digitalWrite(_SDA, HIGH); _delay_us(5);
   digitalWrite(_SCL, HIGH); _delay_us(5);
   digitalWrite(_SDA, LOW); _delay_us(5);
   digitalWrite(_SCL, LOW);_delay_us(5);
  }
}

uint8_t RB_SoftI2CMaster::I2C_Write(uint8_t dat)
{
   pinMode(_SCL,OUTPUT);
   pinMode(_SDA,OUTPUT);
  
    if(_MODE==1)
  {
     _delay_us(150);
   for(uint8_t i =0;i<8;i++)
   {
      if(dat&0x80)
         {
          digitalWrite(_SDA, HIGH);}
      else {
         digitalWrite(_SDA, LOW);}

      dat<<=1;
      digitalWrite(_SCL, HIGH);
      digitalWrite(_SCL, LOW);
    } 
  }
   else 
   {
     for(uint8_t i =0;i<8;i++)
   {
      if(dat&0x80)
         {
          digitalWrite(_SDA, HIGH);}
      else {
         digitalWrite(_SDA, LOW);}
      _delay_us(5);
      dat<<=1;
      digitalWrite(_SCL, HIGH);_delay_us(5);
      digitalWrite(_SCL, LOW);_delay_us(5);
    } 
    
   }
}
uint8_t RB_SoftI2CMaster::I2C_Read(void)
{ 
  uint8_t dat;
  pinMode(_SDA,INPUT);
  if(_MODE==1){
    _delay_us(150);
  for(uint8_t i=0;i<8;i++)
    {
      digitalWrite(_SCL, HIGH);
      dat<<=1;
      if(digitalRead(_SDA)){
         dat|=0x01;
      }
      digitalWrite(_SCL, LOW);
    }
   return   dat;
  }
  else
  {
    for(uint8_t i=0;i<8;i++)
    {
      digitalWrite(_SCL, HIGH);
      dat<<=1;
      if(digitalRead(_SDA)){
         dat|=0x01;
      }
      _delay_us(5);
      digitalWrite(_SCL, LOW);;_delay_us(5);
    }
   return   dat;
   }
  }

  uint8_t RB_SoftI2CMaster::I2C_GetAck(void)
{
   uint8_t ack=0;
   pinMode(_SDA,INPUT);
   if(_MODE==1)
   {
    _delay_us(150);
     //总线准备，接受应答
   digitalWrite(_SDA, HIGH);
   digitalWrite(_SCL, HIGH);
   if(digitalRead(_SDA)!=0){
        ack = 1;
   }
   digitalWrite(_SCL, LOW);
   return ack;
   }
   else
   {
   digitalWrite(_SDA, HIGH);_delay_us(5);
   digitalWrite(_SCL, HIGH);_delay_us(5);
   if(digitalRead(_SDA)!=0){
        ack = 1;
   }
   digitalWrite(_SCL, LOW);_delay_us(5);
   return ack;
   }
}
void RB_SoftI2CMaster::I2C_PutAck(uint8_t ack)
{
   pinMode(_SDA,OUTPUT);
   if(_MODE==1)
   { 
   _delay_us(150);
   if(ack==0)
    {
     digitalWrite(_SDA, LOW);
    }
    else
    {
     digitalWrite(_SDA, HIGH);
    }
    digitalWrite(_SCL, HIGH);
    digitalWrite(_SCL, LOW);
    digitalWrite(_SDA, HIGH);
   }
   else 
    {
      if(ack==0)
    {
     digitalWrite(_SDA, LOW);
    }
    else
    {
     digitalWrite(_SDA, HIGH);
    }
    digitalWrite(_SCL, HIGH);_delay_us(5);
    digitalWrite(_SCL, LOW);_delay_us(5);
    digitalWrite(_SDA, HIGH);
    }
}
void RB_SoftI2CMaster::I2C_Stop(void )
{
  pinMode(_SDA,OUTPUT);
  if(_MODE==1)
  {
  _delay_us(150);
  digitalWrite(_SCL, LOW);
  digitalWrite(_SDA, LOW);
  digitalWrite(_SCL, HIGH);
  digitalWrite(_SDA, HIGH);
  }
  else 
   {
    digitalWrite(_SCL, LOW);_delay_us(5);
  digitalWrite(_SDA, LOW);_delay_us(5);
  digitalWrite(_SCL, HIGH);_delay_us(5);
  digitalWrite(_SDA, HIGH);_delay_us(5);
   }
}
void RB_SoftI2CMaster::beginTransmission(uint8_t slaveaddress)
{
  I2C_Star();
  I2C_Write(slaveaddress<<1|0x00);
  _delay_us(30);
  I2C_GetAck();
}

void RB_SoftI2CMaster::endTransmission(void)
{
  I2C_Stop();
 }
 uint8_t RB_SoftI2CMaster::send(uint8_t data)
{ 
  _delay_us(200);
  I2C_Write(data);
  _delay_us(50);
  I2C_GetAck();
 }
