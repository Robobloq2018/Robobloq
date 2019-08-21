#include "RB_GYRO.h"
#include <util/delay.h>
#include <string.h>
#include "avr/wdt.h"

RB_GYRO::RB_GYRO(void):RB_SoftI2CMaster(0)
{
	
	RB_SoftI2CMaster::SetMode(1);
	
}


RB_GYRO::RB_GYRO(uint8_t port):RB_SoftI2CMaster(port)
{
	RB_SoftI2CMaster::SetMode(1);
	Device_Address = GYRO_ADDRESS_GND;
}



void RB_GYRO::ReadData(uint8_t start_regaddress,uint8_t *buffer,uint8_t datalen)
{
	
	RB_SoftI2CMaster::I2C_Star();
  RB_SoftI2CMaster::I2C_Write(Device_Address<<1|0x00);
  RB_SoftI2CMaster::I2C_GetAck();
  RB_SoftI2CMaster::I2C_Write(start_regaddress);
  RB_SoftI2CMaster::I2C_GetAck();
  RB_SoftI2CMaster::I2C_Stop();
	_delay_us(15);
	RB_SoftI2CMaster::I2C_Star();
	RB_SoftI2CMaster::I2C_Write(Device_Address<<1|0x01);
	RB_SoftI2CMaster::I2C_GetAck();
	datalen=datalen-1;
	while(datalen--)
	{
		*buffer =I2C_Read(); 
		I2C_PutAck(0);
		buffer++;
	}
	*buffer =I2C_Read(); 
	RB_SoftI2CMaster::I2C_PutAck(1);
	RB_SoftI2CMaster::I2C_Stop();
	
}

void RB_GYRO::WriteData(uint8_t start_regaddress,uint8_t *buffer,uint8_t datalen)
{
  
  
  
  RB_SoftI2CMaster::I2C_Star();
  RB_SoftI2CMaster::I2C_Write(Device_Address<<1|0x00);
  RB_SoftI2CMaster::I2C_GetAck();
  RB_SoftI2CMaster::I2C_Write(start_regaddress);
  RB_SoftI2CMaster::I2C_GetAck();
  while(datalen--)
  {
	 RB_SoftI2CMaster::I2C_Write(*buffer);
     RB_SoftI2CMaster::I2C_GetAck();

      buffer++;
  }
  RB_SoftI2CMaster::I2C_Stop();
}

void RB_GYRO::WriteReg(uint8_t start_regaddress,uint8_t buffer)
{
  RB_SoftI2CMaster::I2C_Star();
  RB_SoftI2CMaster::I2C_Write(Device_Address<<1|0x00);
  RB_SoftI2CMaster::I2C_GetAck();
  RB_SoftI2CMaster::I2C_Write(start_regaddress);
  RB_SoftI2CMaster::I2C_GetAck();
  RB_SoftI2CMaster::I2C_Write(buffer);
  RB_SoftI2CMaster::I2C_GetAck();
  RB_SoftI2CMaster::I2C_Stop();
}

uint8_t RB_GYRO::begin(void)
{
	gSensitivity = 65.5 ;
	
	AngleX = 0;
	AngleY = 0;
	AngleZ = 0;
	
	GyroX = 0;
	GyroY = 0;
	GyroZ = 0;
	
	AccX = 0;
	AccY = 0;
	AccZ = 0;
	
	GyroXoffset = 0;
	GyroYoffset = 0;
	GyroZoffset = 0;
	
	delay(100);
	WriteReg(0x6B,0X00);
	delay(30);
	WriteReg(0X1A,0X01);
	WriteReg(0X1B,0X08);
	delay(50);
	deviceCalibration();
}


void RB_GYRO::Update(void)
{
	static unsigned long last_time = 0;
	double dt,filter_coefficient ;
	double ax, ay, az;
	ReadData(0x3b,i2cData,14);
	AccX = ((i2cData[0]<<8)|i2cData[1]);
	AccY = ((i2cData[2]<<8)|i2cData[3]);
	AccZ = ((i2cData[4]<<8)|i2cData[5]);
	GyroX = (((i2cData[8]<<8)|i2cData[9])-GyroXoffset)/gSensitivity;
	GyroY = (((i2cData[10]<<8)|i2cData[11])-GyroYoffset)/gSensitivity;
	GyroZ = (((i2cData[12]<<8)|i2cData[13])-GyroZoffset)/gSensitivity;
	
	ax = atan2(AccX, sqrt( pow(AccY, 2) + pow(AccZ, 2) ) ) * 180 / 3.1415926;
    ay = atan2(AccY, sqrt( pow(AccX, 2) + pow(AccZ, 2) ) ) * 180 / 3.1415926; 
	
	dt = (double)(millis() - last_time) / 1000;
    last_time = millis();

    if(AccZ > 0){
        AngleX = AngleX - GyroY * dt;
        AngleY = AngleY + GyroX * dt;
     }
      else{
         AngleX = AngleX + GyroY * dt;
         AngleY = AngleY - GyroX * dt;
     }
     AngleZ += GyroZ * dt;
     AngleZ = AngleZ - 360 * floor(AngleZ/360);
   

      /*
     complementary filter
     set 0.5sec = tau = dt * A / (1 - A)
     so A = tau / (tau + dt)
      */
        filter_coefficient = 0.5 / (0.5 + dt);
        AngleX = AngleX * filter_coefficient + ax * (1 - filter_coefficient);
        AngleY = AngleY * filter_coefficient + ay * (1 - filter_coefficient);   
	  
}


void RB_GYRO::FastUpdate(void)
{
	
	static unsigned long last_time = 0;
	double dt,filter_coefficient ;
	double ax, ay, az;
	ReadData(0x3b,i2cData,14);
	AccX = ((i2cData[0]<<8)|i2cData[1]);
	AccY = ((i2cData[2]<<8)|i2cData[3]);
	AccZ = ((i2cData[4]<<8)|i2cData[5]);
	GyroX = (((i2cData[8]<<8)|i2cData[9])-GyroXoffset)/gSensitivity;
	GyroY = (((i2cData[10]<<8)|i2cData[11])-GyroYoffset)/gSensitivity;
	GyroZ = (((i2cData[12]<<8)|i2cData[13])-GyroZoffset)/gSensitivity;
	
	ax = atan2(AccX, sqrt( pow(AccY, 2) + pow(AccZ, 2) ) ) * 180 / 3.1415926;
    ay = atan2(AccY, sqrt( pow(AccX, 2) + pow(AccZ, 2) ) ) * 180 / 3.1415926; 
	
	dt = (double)(millis() - last_time) / 1000;
    last_time = millis();

    if(AccZ > 0){
        AngleX = AngleX - GyroY * dt;
        AngleY = AngleY + GyroX * dt;
     }
      else{
         AngleX = AngleX + GyroY * dt;
         AngleY = AngleY - GyroX * dt;
     }
     AngleZ += GyroZ * dt;
     AngleZ = AngleZ - 360 * floor(AngleZ/360);
    
	
      AngleX = AngleX * 0.98 + ax * 0.02;
      AngleY = AngleY * 0.98 + ay * 0.02;   
}

double RB_GYRO::getAngleX(void)
{
	  return AngleX*(-1);
}

double RB_GYRO::getAngleY(void)
{
	  return AngleY*(-1);
}
double RB_GYRO::getAngleZ(void)
{
	  return AngleZ;
}

double RB_GYRO::getGyroX(void)
{
	  return GyroX ;
}

double RB_GYRO::getGyroY(void)
{
	   return GyroY;
}
double RB_GYRO::getGyroZ(void)
{   
	   return GyroZ;
}


int16_t RB_GYRO::getAccX(void)
{
	   return AccX;
}

int16_t RB_GYRO::getAccY(void)
{
	   return AccY;
}
int16_t RB_GYRO::getAccZ(void)
{
	    return AccZ;
}

void RB_GYRO::deviceCalibration(void)
{
	   uint16_t  x = 0;
	   uint16_t  num = 500;
	   long xsum= 0,ysum = 0,zsum = 0;
	   
	   for( x = 0;x<num;x++)
	   {
		   ReadData(0x43,i2cData,6);
		   xsum +=((i2cData[0]<<8)|i2cData[1]);
		   ysum +=((i2cData[2]<<8)|i2cData[3]);
		   zsum +=((i2cData[4]<<8)|i2cData[5]);
       wdt_reset();
	   }
     
	   GyroXoffset = xsum/num;
	   GyroYoffset = ysum/num;
	   GyroZoffset = zsum/num;
 
}
