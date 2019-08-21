#include "RB_COLORSENSOR.h"




RB_COLORSENSOR::RB_COLORSENSOR(uint8_t port,tcs34725IntegrationTime_t it,tcs34725Gain_t gain):RB_SoftI2CMaster(port)
{
   _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain; 
}

uint8_t RB_COLORSENSOR::Read_OneByte(uint8_t slaveaddress,uint8_t regaddress)
 {  

     uint8_t  data= 0;
    RB_SoftI2CMaster::I2C_Star();
    RB_SoftI2CMaster::I2C_Write(slaveaddress<<1|0x00);
   
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Write(regaddress|0x80);

    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Star();
    RB_SoftI2CMaster::I2C_Write(slaveaddress<<1|0x01);

    RB_SoftI2CMaster::I2C_GetAck();
    data = I2C_Read();
    RB_SoftI2CMaster::I2C_PutAck(1);

    RB_SoftI2CMaster::I2C_Stop();

    return  data;
  
  }

  uint16_t RB_COLORSENSOR::Read_TwoByte(uint8_t slaveaddress,uint8_t regaddress)
 {  
    uint16_t  x,t;
    uint16_t  data= 0;
    RB_SoftI2CMaster::I2C_Star();
    RB_SoftI2CMaster::I2C_Write(slaveaddress<<1|0x00);
  
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Write(regaddress|0x80);

    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Star();
    RB_SoftI2CMaster:: I2C_Write(slaveaddress<<1|0x01);

    RB_SoftI2CMaster:: I2C_GetAck();
    t = I2C_Read();
    RB_SoftI2CMaster::I2C_PutAck(0);
     x = I2C_Read();
    RB_SoftI2CMaster::I2C_PutAck(1);
    RB_SoftI2CMaster:: I2C_Stop();
    data = x<<8|t;
    
    return  data;
  
  }


 uint8_t RB_COLORSENSOR::Write_OneByte(uint8_t slaveaddress,uint8_t regaddress,uint8_t data)
 {  

     
    RB_SoftI2CMaster::I2C_Star();
    RB_SoftI2CMaster::I2C_Write(slaveaddress<<1|0x00);
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Write(regaddress|0x80);
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Write(data);
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Stop();
    return  data;
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


 uint16_t  RB_COLORSENSOR::getRGBWvalue(void)
 {
    uint16_t red,green,blue,white;
    uint16_t whitevalue[6];
    uint16_t sum = 0,max=0,min=0;
    uint16_t return_value = 0;
    for(uint8_t i=0;i<6;i++)
    {
    getRawData(&red,&green,&blue,&white);
    whitevalue[i] = white;
    if(whitevalue[i]>=max) 
       max = whitevalue[i];
    if(whitevalue[i]<=min)
       min = whitevalue[i];
    sum += whitevalue[i];
    }
    sum = sum -max - min;
    return_value = (sum/4);
    return return_value;  
  
}


  unsigned short RB_COLORSENSOR::RGBToHSV(float *h,float *s,float *v)
  {   
      uint16_t red,green,blue,white;
      float r,g,b;
      float x;
      getRawData(&red,&green,&blue,&white);
      r = (float)red/white;
      g = (float)green/white;
      b = (float)blue/white;
      
      x = min(min(r,g),b);
      *v = max(max(r,g),b);

      *h = 0;
      *s = 0;

      if(x!=*v)
      {
        float f = (r== x) ? g - b : ((g == x) ? b - r : r - g);
        float i = (r== x) ? 3 : ((g == x) ? 5 : 1);
        *h = ((i - (f / (*v - x))) * 60);
        *h = (float)((int)(*h)%360);
        *s = (*v - x) / *v;
      }
      return white;
  }
  unsigned char RB_COLORSENSOR::GetColor(void)
  {
    uint16_t  red,green,blue,white;
    unsigned char color_value ;
    float hsv_r,hsv_g,hsv_b;
    float x;
    float dir ;
    float h,s,v;
    float p;

    getRawData(&red,&green,&blue,&white);
     
    p = (255.0 / white) * 2.897727272727;
    red = red * p;
    green = green * p;
    blue = blue * p;
    //******************hsv
    hsv_r = red/255.0;
    hsv_g = green/255.0;
    hsv_b = blue/255.0;
    x = min(min(hsv_r,hsv_g),hsv_b);
    v = max(max(hsv_r,hsv_g),hsv_b);

    h = 0;
    s = 0;
      
    if(x!=v)
    {
        float f = (hsv_r== x) ? hsv_g - hsv_b : ((hsv_g == x) ? hsv_b - hsv_r : hsv_r - hsv_g);
        float i = (hsv_r== x) ? 3 : ((hsv_g == x) ? 5 : 1);
        h = ((i - (f / (v - x))) * 60);
        h = (float)((int)(h)%360);
        s = (v - x) / v;
     }
     //*******************CheckColor
     s = s *255.0;
     v = v *255.0;
    /*
    0: 无
    1：红
    2：绿
    3：蓝
    4：黄
    5：紫
    6：白
    */
     dir = (350- (white+red*0.3+green*0.3+blue*0.4));
     if (dir < 80){
          if (v > 255){
           if (20 < h && h < 50 && 80 < s) return 4;
           if (0 < h && h <= 13 || (0 == h && v > 120)) return 1;
           if (345 < h && h < 360) return 1;
           if (220 < h && h < 345) return 5;
           if (160 < h && h <= 220) return blue;
           if (h > 50 && h <= 80 && dir <= 60 && v < 270) return 6;
           if (h > 50 && h <= 127 && v > 270) return 2;
                
           }
           if (v < 270){
              if (h > 15 && h <= 30 && dir <= 60) return 6;
              if (220 < h && h < 345 || h <= 60) return 5;
           }
       }
	  if (dir < 97){ 
	       if (h > 50 && h <= 127 && v > 270) return 2;
	  }
       return 0;
    }

   unsigned char RB_COLORSENSOR::GetColor_Game(void)
  {
     
    uint16_t  red,green,blue,white;  
    uint16_t  color_value=0;	
    getRawData(&red,&green,&blue,&white);
    
    if((red>40)&&(green<40)&&(blue<40)&&((red-green)>25)){
        color_value = 1;    //红
    }        
    if((green>45)&&(red<45)&&(blue<45)&&((green-red)>15)){
        color_value = 2;    //绿
    }
    if((blue>30)&&(red<35)&&(green<35)&&((blue-green)>3)){
        color_value = 3;    //蓝 
    } 
    return  color_value;
   }
	
        
