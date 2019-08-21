#ifndef RB_SoftI2CMaster_h
#define RB_SoftI2CMaster_h

#include <inttypes.h>
#include "RB_PORT.h" 

#define _SOFTI2CMASTER_VERSION 12  // software version of this library


class RB_SoftI2CMaster :public RB_Port
{
public:
    RB_SoftI2CMaster(uint8_t _sdapin,uint8_t _sclpin);
    RB_SoftI2CMaster(uint8_t port);
    void SetMode(uint8_t mode);
    
 //   RB_SoftI2CMaster(uint8_t port,uint8_t mode);
    void I2C_Star(void);
    uint8_t I2C_Write(uint8_t dat);
    uint8_t I2C_Read(void);
    uint8_t I2C_GetAck(void);
    void I2C_PutAck(uint8_t ack);
    void I2C_Stop(void);
    void beginTransmission(uint8_t slaveaddress);
    void endTransmission(void);
    uint8_t send(uint8_t data);
 private:
    uint8_t _SDA;
    uint8_t _SCL;   
    uint8_t _MODE;
};

#endif
