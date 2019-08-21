
#ifndef RB_TOUCHSENSOR_H
#define RB_TOUCHSENSOR_H

#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 
#include "RB_SOFTI2CMASTER.h"


 class RB_TOUCHSENSOR : public RB_SoftI2CMaster
{
  public : 
     RB_TOUCHSENSOR(void);
     RB_TOUCHSENSOR(uint8_t port );
     void      RB_TOUCHSENSOR_ReadArray(uint8_t Register_Address,uint8_t *data,uint8_t datalen);
     uint16_t  RB_TOUCHSENSOR_ReadValue(void);

  private:
     uint16_t touchvalue;
  
};
#endif 
