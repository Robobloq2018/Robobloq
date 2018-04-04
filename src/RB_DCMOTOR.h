#ifndef RB_DCMotor_H
#define RB_DCMotor_H

#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 
#include "RB_SOFTI2CMASTER.h"


 class RB_DCMotor : public RB_SoftI2CMaster
{
  public : 
        RB_DCMotor(void);
		RB_DCMotor(uint8_t port);
		void RB_DCMotor_SetSpeed(uint8_t SlaveAddress,uint8_t RegisterAddress,uint8_t Index,uint8_t Dir,uint8_t Speed);
   
};
#endif 
