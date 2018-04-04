#ifndef RB_LEDMatrix_H
#define RB_LEDMatrix_H

#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 
#include "RB_SOFTI2CMASTER.h"

#define TM1680ID       0xe7
#define SYSDIS         0x80
#define SYSEN          0x81
#define LEDOFF         0x82
#define LEDON          0x83
#define BLINKOFF       0x88
#define BLINK2HZ       0x89
#define BLINK1HZ       0x8A
#define BLINK0_5HZ     0x8B

#define SLAVEMODE     0x90
#define RCMODE0       0x98
#define RCMODE1       0x9A   
#define EXTCLK0       0x9C
#define EXTCLK1       0x9E

#define COM8NMOS      0xA0
#define COM16NMOS     0xA4     //com 16N 
#define COM8PMOS      0xA8
#define COM16PMOS     0xAC

#define PWM01         0xB0
#define PWM02         0xB1
#define PWM03         0xB2
#define PWM04         0xB3
#define PWM05         0xB4
#define PWM06         0xB5
#define PWM07         0xB6
#define PWM08         0xB7
#define PWM09         0xB8
#define PWM10         0xB9
#define PWM11         0xBA
#define PWM12         0xBB
#define PWM13         0xBC
#define PWM14         0xBD
#define PWM15         0xBE
#define PWM16         0xBF

 class RB_LEDMatrix : public RB_SoftI2CMaster
{
    public :
      RB_LEDMatrix(void);
      RB_LEDMatrix(uint8_t port);
      void RB_LEDMatrix_WriteOneByte(unsigned char faddr,unsigned char sdate);
      void RB_LEDMatrix_PageAllWrite(unsigned char faddr,unsigned char sdate,unsigned char cnt);
      void RB_LEDMatrix_PageWrite(unsigned char faddr, unsigned char pdate[20]) ;
      void RB_LEDMatrix_DataChange(unsigned char *inputdata,unsigned char *outputdata);
        
};

#endif
