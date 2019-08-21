
#ifndef RB_LINEFOLLOWARRAY_H
#define RB_LINEFOLLOWARRAY_H

#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 
#include "RB_SOFTI2CMASTER.h"


 class RB_LINEFOLLOWARRAY : public RB_SoftI2CMaster
{
  public : 
     RB_LINEFOLLOWARRAY(void);
     RB_LINEFOLLOWARRAY(uint8_t port );
     void RB_LINEFOLLOWARRAY::RB_LINEFOLLOWARRAY_ReadArray(uint8_t Register_Address,uint8_t *data,uint8_t datalen);
     uint8_t  RB_LINEFOLLOWARRAY::RB_LINEFOLLOWARRAY_ReadValue(void);
  private:
     uint8_t Last_Arry ;
  
};
#endif 
