#ifndef  RB_GYRO_H_
#define  RB_GYRO_H_


#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 
#include "RB_SOFTI2CMASTER.h"

#define GYRO_ADDRESS_GND 0X68;
#define GYRO_ADDRESS_VCC 0X69;

class RB_GYRO : public RB_SoftI2CMaster
{
  public : 
        RB_GYRO(void);
		RB_GYRO(uint8_t port);
		void ReadData(uint8_t start_regaddress,uint8_t *buffer,uint8_t datalen);
		void WriteData(uint8_t start_regaddress,uint8_t *buffer,uint8_t datalen);
		void WriteReg(uint8_t start_regaddress,uint8_t buffer);
		uint8_t begin(void);
		void Update(void);
		void FastUpdate(void);
		double RB_GYRO::getAngleX(void);
		double RB_GYRO::getAngleY(void);
		double RB_GYRO::getAngleZ(void);
		double RB_GYRO::getGyroX(void);
		double RB_GYRO::getGyroY(void);
		double RB_GYRO::getGyroZ(void);
		int16_t RB_GYRO::getAccX(void);
		int16_t RB_GYRO::getAccY(void);
    int16_t RB_GYRO::getAccZ(void);
		
  private:
        double gSensitivity ;
		double AngleX,AngleY,AngleZ;
		double GyroX,GyroY,GyroZ;
		int16_t AccX,AccY,AccZ;
		double GyroXoffset,GyroYoffset,GyroZoffset;
		uint8_t i2cData[14];
		uint16_t Device_Address;
		void deviceCalibration(void);
  

};

#endif
