#ifndef RB_DIGITALDISPLAY_H
#define RB_DIGITALDISPLAY_H

#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 
#include "RB_SOFTI2CMASTER.h"


 class RB_DIGITALDISPLAY : public RB_SoftI2CMaster
{
  public : 
        RB_DIGITALDISPLAY(void);
		RB_DIGITALDISPLAY(uint8_t port);
        void RB_DIGITALDISPLAY_Init(void);
		void RB_DIGITALDISPLAY_SET(uint8_t Address,uint8_t Data);
        void Clear(void);
};
#endif 
