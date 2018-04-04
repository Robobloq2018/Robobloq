#ifndef  RB_ADC_H
#define  RB_ADC_H

#include <inttypes.h>
#include <Arduino.h>


#include "RB_PORT.h" 

class  RB_ADC
{ 
    public :
    RB_ADC(void);
    uint16_t RB_ADC_Read(uint8_t port);
    uint16_t ADC_Read(void);
  
};
#endif 
