#include "RB_COLORSENSOR.h"




RB_COLORSENSOR::RB_COLORSENSOR(uint8_t port,tcs34725IntegrationTime_t it,tcs34725Gain_t gain):RB_SoftI2CMaster(port)
{
   _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain; 
}



 
 bool   RB_COLORSENSOR:: begin(void)
  { 
    unsigned char x;
   
    x= Read_OneByte(TCS34725_ADDRESS,TCS34725_ID);

    if((x!=0x44)&&(x!=0x10))
    {
      return false;
    }
    _tcs34725Initialised = true;
    setIntegrationTime(_tcs34725IntegrationTime);
    setGain(_tcs34725Gain);
    enable();
    return true;
  }
  

  void   RB_COLORSENSOR::  setIntegrationTime(tcs34725IntegrationTime_t it)
  {
if (!_tcs34725Initialised) begin();

 
  Write_OneByte(TCS34725_ADDRESS,TCS34725_ATIME, it);

  _tcs34725IntegrationTime = it;
    
    
  }
  void   RB_COLORSENSOR::  setGain(tcs34725Gain_t gain)
  {
    if (!_tcs34725Initialised) begin();

 
  Write_OneByte(TCS34725_ADDRESS,TCS34725_CONTROL, gain);


  _tcs34725Gain = gain;
    
    
  }


  void   RB_COLORSENSOR::  getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
  {
      
    
    if (!_tcs34725Initialised) 
        {begin();}
       
  *c = Read_TwoByte(TCS34725_ADDRESS,TCS34725_CDATAL);
  
  *r = Read_TwoByte(TCS34725_ADDRESS,TCS34725_RDATAL);

  *g = Read_TwoByte(TCS34725_ADDRESS,TCS34725_GDATAL);
  
  *b = Read_TwoByte(TCS34725_ADDRESS,TCS34725_BDATAL);

   
  switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      delay(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      delay(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      delay(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS:
      delay(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      delay(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS:
      delay(700);
      break;
    
    
  }
  }

  uint16_t RB_COLORSENSOR:: calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
  {
    float X, Y, Z;     
  float xc, yc;      
  float n;           
  float cct;

 
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);


  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);


  n = (xc - 0.3320F) / (0.1858F - yc);

  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  return (uint16_t)cct;
    
    
  }
  uint16_t RB_COLORSENSOR:: calculateLux(uint16_t r, uint16_t g, uint16_t b)
  {
    
   float illuminance;


  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance; 
    
  }
  
  void RB_COLORSENSOR:: setInterrupt(boolean i)
  {
    uint8_t r ;
    r= Read_OneByte(TCS34725_ADDRESS,TCS34725_ENABLE);
    
      if (i) {
         r |= TCS34725_ENABLE_AIEN;
      } else {
        r &= ~TCS34725_ENABLE_AIEN;
      }
      Write_OneByte(TCS34725_ADDRESS,TCS34725_ENABLE, r); 
    
  }
  void RB_COLORSENSOR:: clearInterrupt(void)
  {

    
  }
  void RB_COLORSENSOR:: setIntLimits(uint16_t low, uint16_t high)
  {
   Write_OneByte(TCS34725_ADDRESS,0x04, low & 0xFF);
   Write_OneByte(TCS34725_ADDRESS,0x05, low >> 8);
   Write_OneByte(TCS34725_ADDRESS,0x06, high & 0xFF);
   Write_OneByte(TCS34725_ADDRESS,0x07, high >> 8);  
    
  }
  void RB_COLORSENSOR::enable(void)
  {
  Write_OneByte(TCS34725_ADDRESS,TCS34725_ENABLE, TCS34725_ENABLE_PON);
  delay(3);
  Write_OneByte(TCS34725_ADDRESS,TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);    
  }
  
  void RB_COLORSENSOR::disable(void)
{
  
  uint8_t reg = 0;
  reg = Read_OneByte(TCS34725_ADDRESS,TCS34725_ENABLE);
  Write_OneByte(TCS34725_ADDRESS,TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
  }
